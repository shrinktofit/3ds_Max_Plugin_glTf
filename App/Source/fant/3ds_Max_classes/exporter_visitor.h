
#pragma once

#include <array>
#include <charconv>
#include <decomp.h>
#include <fant/3ds_Max_classes/exporter.h>
#include <fant/3ds_Max_classes/exporter/export_settings.h>
#include <fant/glTF.h>
#include <fant/support/win32/get_string_resource.h>
#include <fant/support/win32/instance.h>
#include <fant/support/win32/mchar_to_utf8.h>
#include <filesystem>
#include <fstream>
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
#include <vector>

namespace fant {
inline bool undo_parents_offset(INode &node_, Point3 &point_, Quat &offset_rotation_) {
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

class exporter_visitor : public ITreeEnumProc {
  using _fpi_type = std::common_type_t<TimeValue, unsigned char>;

public:
  exporter_visitor(Interface &max_interface_,
                   glTF::document &document_,
                   const export_settings &settings_);

  int callback(INode *max_node_) override;

  glTF::object_ptr<glTF::node> _convertNode(INode &max_node_);

  glTF::object_ptr<glTF::mesh> _convertTriObj(TriObject &tri_obj_);

  void _bakeAnimation(INode &max_node_,
                      TimeValue start_time_,
                      TimeValue step_,
                      TimeValue frame_count_,
                      glTF::object_ptr<glTF::node> glTF_node_,
                      glTF::object_ptr<glTF::accessor> input_);

  void _writePoint3(float *output_, const Point3 &p_) {
    output_[0] = p_.x;
    output_[1] = p_.y;
    output_[2] = p_.z;
  }

  void _writeQuat(float *output_, const Quat &q_) {
    output_[0] = q_.x;
    output_[1] = q_.y;
    output_[2] = q_.z;
    output_[2] = q_.w;
  }

  Matrix3 _getObjectOffsetTM(INode &node_) {
    Matrix3 tm(1);
    Point3 pos = node_.GetObjOffsetPos();
    tm.PreTranslate(pos);
    Quat quat = node_.GetObjOffsetRot();
    PreRotateMatrix(tm, quat);
    ScaleValue scaleValue = node_.GetObjOffsetScale();
    ApplyScaling(tm, scaleValue);
    return tm;
  }

  glTF::object_ptr<glTF::mesh>
  _addMesh(std::u8string_view name_,
           const vertex_list &vertex_list_,
           gsl::span<vertex_list::size_type> indices_);

  glTF::object_ptr<glTF::accessor>
  _addIndices(gsl::span<vertex_list::size_type> indices_);

  glTF::object_ptr<glTF::animation> _getOrCreateGlTFAnimation() {
    return nullptr;
  }

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
  std::unordered_map<INode *, glTF::object_ptr<glTF::node>> _nodeMaps;
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

  template <typename Out>
  static void _flatternVec3Array(gsl::span<Point3> values_, Out *out_) {
    for (decltype(values_.size()) i = 0; i < values_.size(); ++i) {
      out_[3 * i + 0] = values_[i].x;
      out_[3 * i + 1] = values_[i].y;
      out_[3 * i + 2] = values_[i].z;
    }
  }

  template <typename Out>
  static void _flatternQuatArray(gsl::span<Quat> values_, Out *out_) {
    for (decltype(values_.size()) i = 0; i < values_.size(); ++i) {
      out_[4 * i + 0] = values_[i].x;
      out_[4 * i + 1] = values_[i].y;
      out_[4 * i + 2] = values_[i].z;
      out_[4 * i + 3] = values_[i].w;
    }
  }

  glTF::object_ptr<glTF::accessor>
  _makeSimpleAccessor(glTF::accessor::type_type type_,
                      glTF::accessor::component_type comp_type_,
                      glTF::integer count_) {
    auto nBytes = glTF::accessor::required_bytes(comp_type_) *
                  glTF::accessor::required_components(type_);
    auto bufferView =
        _document.factory().make<glTF::buffer_view>(_mainBuffer, // buffer
                                                    nBytes,      // size
                                                    0            // alignment
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

  glTF::object_ptr<glTF::accessor>
  _makeSimpleAccessor(gsl::span<Point3> values_) {
    auto accessor = _makeSimpleAccessor(
        glTF::accessor::type_type::vec3,
        glTF::accessor::component_type::the_float, values_.size());
    _flatternVec3Array(
        values_,
        accessor->typed_data<glTF::accessor::component_type::the_float>());
    return accessor;
  }

  glTF::object_ptr<glTF::accessor>
  _makeSimpleAccessor(gsl::span<Quat> values_) {
    auto accessor = _makeSimpleAccessor(
        glTF::accessor::type_type::vec3,
        glTF::accessor::component_type::the_float, values_.size());
    _flatternQuatArray(
        values_,
        accessor->typed_data<glTF::accessor::component_type::the_float>());
    return accessor;
  }

  static Matrix3 _getLocalNodeTransformMatrix(INode &max_node_,
                                              TimeValue time_) {
    auto worldTM = max_node_.GetNodeTM(time_);
    if (!max_node_.GetParentNode()->IsRootNode()) {
      auto inverseParent = Inverse(max_node_.GetParentNode()->GetNodeTM(time_));
      return worldTM * inverseParent;
    } else {
      return worldTM;
    }
  }
};
} // namespace fant