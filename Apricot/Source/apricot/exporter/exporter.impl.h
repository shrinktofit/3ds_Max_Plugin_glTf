
#pragma once

#define DEBUG_TPOSE

#include <IGame/IGame.h>
#include <apricot/exporter/export_settings.h>
#include <apricot/glTF.h>
#include <array>
#include <charconv>
#include <decomp.h>
#include <filesystem>
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <gsl/span>
#include <iskin.h>
#include <list>
#include <memory>
#include <modstack.h>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

// Forward declaration for Max::StdMat
class StdMat;
class MultiMtl;

namespace apricot {
inline bool
undo_parents_offset(INode &node_, Point3 &point_, Quat &offset_rotation_) {
  auto parent = node_.GetParentNode();
  if (parent->IsRootNode()) {
    return false;
  }
  auto parentOffsetRotation = parent->GetObjOffsetRot();
  if (parentOffsetRotation == IdentQuat()) {
    return false;
  }
  Matrix3 mat(true);
  parentOffsetRotation.MakeMatrix(mat);
  mat = Inverse(mat);
  point_ = VectorTransform(mat, point_);
  offset_rotation_ = offset_rotation_ / parentOffsetRotation;
  return true;
}

class exporter_impl {
  using _fpi_type = std::common_type_t<TimeValue, unsigned char>;

public:
  exporter_impl(Interface &max_interface_,
                glTF::document &document_,
                const export_settings &settings_,
                IScene &scene_);

private:
  struct _immediate_mesh {
    constexpr static auto joint_storage =
        glTF::accessor::component_type::unsigned_short;

    using joint_storage_type =
        glTF::accessor::component_storage_t<joint_storage>;

    constexpr static auto weight_storage =
        glTF::accessor::component_type::the_float;

    using weight_storage_type =
        glTF::accessor::component_storage_t<weight_storage>;

    std::u8string name;
    struct vertex_list {
    public:
      vertex_list &operator=(vertex_list &&) = default;
      vertex_list(vertex_list &&) = default;
      vertex_list &operator=(const vertex_list &) = delete;
      vertex_list(const vertex_list &) = delete;

      using vertex_count_type = glTF::buffer::size_type;

      struct channel_type {
        using element_size_type = unsigned;
        const glTF::accessor::type_type type;
        const glTF::accessor::component_type component;
        const element_size_type element_size;
        std::unique_ptr<std::byte[]> data;

        channel_type(glTF::accessor::type_type type_,
                     glTF::accessor::component_type component_,
                     element_size_type element_size_,
                     vertex_count_type vertex_count_)
            : type(type_), component(component_), element_size(element_size_),
              _attributeBytes(glTF::accessor::required_bytes(component_) *
                              glTF::accessor::required_components(type_) *
                              element_size_) {
          data = std::make_unique<std::byte[]>(_attributeBytes * vertex_count_);
        }

        auto attribute_bytes() const {
          return _attributeBytes;
        }

      private:
        glTF::integer _attributeBytes;
      };

      vertex_list(vertex_count_type vertex_count_) : _nVertices(vertex_count_) {
      }

      vertex_count_type vertex_count() const {
        return _nVertices;
      }

      auto channels_begin() const {
        return _channels.cbegin();
      }

      auto channels_end() const {
        return _channels.cend();
      }

      auto channels_size() const {
        return _channels.size();
      }

      std::byte *
      add_channel(std::u8string_view channel_name_,
                  glTF::accessor::type_type type_,
                  glTF::accessor::component_type component_,
                  channel_type::element_size_type element_size_ = 1) {
        auto attributesByte = glTF::accessor::required_bytes(component_) *
                              glTF::accessor::required_components(type_);
        auto r = _channels.emplace(
            channel_name_,
            channel_type{type_, component_, element_size_, _nVertices});
        return r.first->second.data.get();
      }

      std::pair<vertex_list, std::vector<vertex_count_type>> to_indexed() const;

    private:
      std::map<std::u8string, channel_type> _channels;
      vertex_count_type _nVertices;
    };

    struct submesh {
      using indices_type = std::vector<vertex_list::vertex_count_type>;

      submesh(vertex_list &&vertices_)
          : material_id(0), vertices(std::move(vertices_)) {
      }

      submesh(vertex_list &&vertices_, indices_type &&indices_)
          : material_id(0), vertices(std::move(vertices_)),
            indices(std::move(indices_)) {
      }

      submesh(MtlID material_id_, vertex_list &&vertices_)
          : material_id(material_id_), vertices(std::move(vertices_)) {
      }

      submesh(MtlID material_id_,
              vertex_list &&vertices_,
              indices_type &&indices_)
          : material_id(material_id_), vertices(std::move(vertices_)),
            indices(std::move(indices_)) {
      }

      MtlID material_id;
      std::optional<indices_type> indices;
      vertex_list vertices;
    };

    std::vector<submesh> submeshes;
    std::vector<int> vertex_resort;

    _immediate_mesh() = delete;

    _immediate_mesh(std::u8string &&name_, std::vector<submesh> &&submeshes_)
        : name(std::move(name_)), submeshes(std::move(submeshes_)) {
    }
  };

  struct skin_statistics {
    using influence_size_type =
        _immediate_mesh::vertex_list::channel_type::element_size_type;

    using vertex_count_type = _immediate_mesh::vertex_list::vertex_count_type;

    using joint_storage_type = _immediate_mesh::joint_storage_type;

    using weight_storage_type = _immediate_mesh::weight_storage_type;

    skin_statistics(influence_size_type influence_count_,
                    vertex_count_type vertex_count_)
        : influence_count(influence_count_),
          joint_channel(std::make_unique<joint_storage_type[]>(
              influence_count_ * vertex_count_)),
          weight_channel(std::make_unique<weight_storage_type[]>(
              influence_count_ * vertex_count_)) {
    }

    void set_influence(vertex_count_type vertex_index_,
                       influence_size_type influnce_index_,
                       joint_storage_type joint_,
                       weight_storage_type weight_) {
      auto i = influence_count * vertex_index_ + influnce_index_;
      joint_channel[i] = joint_;
      weight_channel[i] = weight_;
    }

    const _immediate_mesh::vertex_list::channel_type::element_size_type
        influence_count;
    const std::unique_ptr<_immediate_mesh::joint_storage_type[]> joint_channel;
    const std::unique_ptr<_immediate_mesh::weight_storage_type[]>
        weight_channel;
#ifdef DEBUG_TPOSE
    std::vector<glm::mat4> bindposes;
    std::vector<glm::mat4> joint_transforms;
#endif
  };

  glTF::object_ptr<glTF::node> _exportNode(IGameNode &igame_node_);

  std::optional<_immediate_mesh>
  _exportMesh(IGameNode &igame_node_,
              IGameMesh &igame_mesh_,
              const std::optional<skin_statistics> &skin_data_);

  std::optional<_immediate_mesh> _exportMeshIndexed(IGameNode &igame_node_,
                                                    IGameMesh &igame_mesh_);

  glTF::object_ptr<glTF::mesh>
  _convertMesh(const _immediate_mesh &imm_mesh_,
               const std::vector<glTF::object_ptr<glTF::material>> &materials_);

  std::pair<glTF::object_ptr<glTF::skin>, skin_statistics>
  _exportSkin(IGameNode &igame_node_,
              IGameSkin &igame_skin_,
              glTF::object_ptr<glTF::node> glTF_mesh_node_);

  std::vector<glTF::object_ptr<glTF::material>>
  _exportMaterial(IGameMaterial &igame_materail_);

  std::vector<glTF::object_ptr<glTF::material>>
  _tryExportMaterial(INode &max_node_);

  std::vector<glTF::object_ptr<glTF::material>>
  _tryConvertMaterial(Mtl &max_mtl_);

  glTF::object_ptr<glTF::material> _convertStdMaterial(StdMat &max_mtl_);

  std::vector<glTF::object_ptr<glTF::material>>
  _convertMultiMaterial(MultiMtl &max_mtl_);

  std::optional<glTF::texture_info> _tryConvertTexture(Texmap &tex_map_);

  void _bakeAnimation(IGameNode &igame_node_,
                      TimeValue start_time_,
                      TimeValue step_,
                      TimeValue frame_count_,
                      glTF::object_ptr<glTF::node> glTF_node_,
                      glTF::object_ptr<glTF::accessor> input_);

  template <typename Out> void _convert(const GMatrix &value_, Out *out_) {
    // glTF expect the matrices to be column vector and stored in column majar;
    // In 3ds Max, all vectors are assumed to be row vectors.
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        // We actually done the transpose: it should have been `out[c * 4 +
        // r]`(column major).
        out_[r * 4 + c] = value_.GetRow(r)[c];
      }
    }
  }

  glTF::object_ptr<glTF::accessor>
  _addIndices(gsl::span<const _immediate_mesh::vertex_list::vertex_count_type>
                  indices_);

  static bool _approxEqual(float lhs_, float rhs_) {
    return std::fabs(lhs_ - rhs_) < 1.0e-5f;
  }

  static bool _approxEqual(const Point3 &lhs_, const Point3 &rhs_) {
    return _approxEqual(lhs_.x, rhs_.x) && _approxEqual(lhs_.y, rhs_.y) &&
           _approxEqual(lhs_.z, rhs_.z);
  }

  static bool _approxEqual(const Quat &lhs_, const Quat &rhs_) {
    return _approxEqual(lhs_.x, rhs_.x) && _approxEqual(lhs_.y, rhs_.y) &&
           _approxEqual(lhs_.z, rhs_.z) && _approxEqual(lhs_.w, rhs_.w);
  }

private:
  Interface &_maxInterface;
  glTF::document &_document;
  const export_settings &_settings;
  std::unordered_map<IGameNode *, glTF::object_ptr<glTF::node>> _nodeMaps2;
  std::unordered_map<Mtl *, std::vector<glTF::object_ptr<glTF::material>>>
      _materialMap;
  std::unordered_map<Texmap *, glTF::texture_info> _textureMap;
  glTF::object_ptr<glTF::scene> _glTFScene;
  glTF::object_ptr<glTF::buffer> _mainBuffer;
  glTF::object_ptr<glTF::animation> _mainAnimation;
  struct _anim_baking {
    TimeValue start;
    TimeValue step;
    TimeValue frame_count;
    glTF::object_ptr<glTF::accessor> input;
  };
  std::optional<_anim_baking> _animBaking;
  std::vector<
      std::pair<std::vector<TimeValue>, glTF::object_ptr<glTF::accessor>>>
      _animationTimesAccessors;

  static GMatrix _convertIntoGlTFAxisSystem(const GMatrix &max_transform_);

  static Matrix3 _convertIntoGlTFAxisSystem(const Matrix3 &max_transform_);

  static Point3 _convertIntoGlTFAxisSystem(const Point3 &max_point_);

  static Quat _convertIntoGlTFAxisSystem(const Quat &max_point_);

  /// <summary>
  /// <c>quat_</c> shall be normalized.
  /// </summary>
  static Quat _igameToGlTF(const Quat &quat_);

  static std::u8string _convertMaxName(const MSTR &max_name_);

  static std::filesystem::path _convertMaxPath(const MSTR &max_path_);

  static Matrix3 _calcOffsetTransformMatrix(INode &max_node_);

  template <typename Out>
  static void _convertMaxMatrix3ToGlTFMat4(const Matrix3 &matrix, Out *out_) {
    // glTF expect the matrices to be column vector and stored in column majar;
    // In 3ds Max, all vectors are assumed to be row vectors.
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 3; ++c) {
        // We actually done the transpose: it should have been `out[c * 4 +
        // r]`(column major).
        out_[r * 4 + c] = matrix.GetRow(r)[c];
      }
      out_[r * 4 + 3] = 0;
    }
    out_[15] = 1;
  }

  void _setTrs(glTF::node &node_, const GMatrix &matrix_);

  glTF::object_ptr<glTF::accessor>
  _makeSimpleAccessor(glTF::accessor::type_type type_,
                      glTF::accessor::component_type comp_type_,
                      glTF::accessor::size_type count_) {
    auto componentBytes = glTF::accessor::required_bytes(comp_type_);
    auto nBytes = glTF::accessor::required_bytes(comp_type_) *
                  glTF::accessor::required_components(type_) * count_;
    auto bufferView =
        _document.factory().make<glTF::buffer_view>(_mainBuffer,   // buffer
                                                    nBytes,        // size
                                                    componentBytes // alignment
        );
    auto accessor =
        _document.factory().make<glTF::accessor>(bufferView, // buffer view
                                                 0,          // offset
                                                 comp_type_, // component type
                                                 type_,      // type
                                                 count_      // count
        );
    return accessor;
  }

  static Matrix3 _getLocalNodeTransformMatrix(INode &max_node_,
                                              TimeValue time_);

  static glm::mat4 _getObjectOffsetTransformMatrix(IGameNode &igame_node_,
                                                   TimeValue time_);

  static Point3 _transformPoint(const GMatrix &matrix_, const Point3 &point_);

  static Point3 _transformVector(const GMatrix &matrix_, const Point3 &point_);

  static glm::mat4 _calculateLocalMatrix(glTF::object_ptr<glTF::node> node_);

  static glm::mat4 _calculateWorldMatrix(glTF::object_ptr<glTF::node> node_);

  struct _trs {
    glm::vec3 translation;
    glm::quat rotation;
    glm::vec3 scale;
  };

  static _trs _decomposeTRS(const glm::mat4 &matrix_);

  static glm::mat4 _composeTRS(const glm::vec3 &translation_,
                               const glm::quat &rotation_,
                               const glm::vec3 &scale_);

  static glm::mat4 _toGLM(const GMatrix &matrix_);

  static glm::vec3 _toGLM(const Point3 &matrix_);

  static glm::quat _toGLM(const Quat &quat_);

  struct _mathconv_max_to_glTF {
    static glm::vec3 convert_point(const glm::vec3 &point_);

    static glm::vec3 convert_scale(const glm::vec3 &scale_);

    static glm::quat convert_rotation(const glm::quat &quat_);

    static glm::mat4 convert_transform(const glm::mat4 &matrix_);
  };

  struct _mathconv_igame_to_glTF {
    static glm::quat convert_rotation(const glm::quat &quat_);
  };
};
} // namespace apricot