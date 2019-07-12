
#define NOMINMAX
#include "resource.h"
#include <array>
#include <charconv>
#include <filesystem>
#include <fstream>
#include <fx/gltf.h>
#include <gsl/span>
#include <list>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <string_view>
#include <tapu/3ds_Max_classes/exporter.h>
#include <tapu/support/win32/get_string_resource.h>
#include <tapu/support/win32/instance.h>
#include <tapu/support/win32/mchar_to_utf8.h>
#include <unordered_map>
#include <vector>

namespace tapu {
enum class glTf_extension {
  gltf,
  glb,
};

struct glTf_extension_info {
  glTf_extension extension;
  const MCHAR *rep;
};

std::array<glTf_extension_info, 2> glTf_extension_info_list{
    glTf_extension_info{glTf_extension::gltf, _M("GLTF")},
    glTf_extension_info{glTf_extension::glb, _M("GLB")}};

class vertex_list {
public:
  using size_type = std::uint32_t;

  using map_channel_size_type = std::uint32_t;

  class vertex_attribute_viewer;

  class const_vertex_attribute_viewer {
    friend class vertex_attribute_viewer;

  public:
    const_vertex_attribute_viewer(const vertex_list &list_, size_type index_)
        : _list(const_cast<vertex_list &>(list_)), _index(index_) {
      assert(index_ < list_._nVertices);
    }

    const Point3 &vertex() const {
      return _list._vertexChannel[_index];
    }

    const Point3 &normal() const {
      return _list._normalChannel[_index];
    }

    const Point3 &texcoord() const {
      return _list._texcoordChannel[_index];
    }

    const Point3 &uvw(map_channel_size_type map_index_) const {
      return _list._mapChannels[map_index_][_index];
    }

    bool operator==(const const_vertex_attribute_viewer &other_) const {
      assert(_list.map_channel_size() == other_._list.map_channel_size());
      if (vertex() != other_.vertex() || normal() != other_.normal() ||
          texcoord() != other_.texcoord()) {
        return false;
      }
      for (map_channel_size_type iMap = 0; iMap < _list.map_channel_size();
           ++iMap) {
        if (uvw(iMap) != other_.uvw(iMap)) {
          return false;
        }
      }
      return true;
    }

  private:
    size_type _index;
    vertex_list &_list;
  };

  class vertex_attribute_viewer : public const_vertex_attribute_viewer {
  public:
    vertex_attribute_viewer(vertex_list &list_, size_type index_)
        : const_vertex_attribute_viewer(list_, index_) {
    }

    void vertex(const Point3 &value_) {
      _list._vertexChannel[_index] = value_;
    }

    void normal(const Point3 &value_) {
      _list._normalChannel[_index] = value_;
    }

    void texcoord(const Point3 &value_) {
      _list._texcoordChannel[_index] = value_;
    }

    void uvw(map_channel_size_type map_index_, const Point3 &value_) {
      _list._mapChannels[map_index_][_index] = value_;
    }

    vertex_attribute_viewer &
    operator=(const const_vertex_attribute_viewer &other_) {
      assert(_list.map_channel_size() == other_._list.map_channel_size());
      vertex(other_.vertex());
      normal(other_.normal());
      texcoord(other_.texcoord());
      for (map_channel_size_type iMap = 0; iMap < _list.map_channel_size();
           ++iMap) {
        uvw(iMap, other_.uvw(iMap));
      }
      return *this;
    }
  };

  vertex_list(size_type size_, map_channel_size_type map_channel_count_)
      : _nVertices(size_), _nMaps(map_channel_count_) {
    _vertexChannel = std::make_unique<Point3[]>(_nVertices);
    _normalChannel = std::make_unique<Point3[]>(_nVertices);
    _texcoordChannel = std::make_unique<Point3[]>(_nVertices);

    _mapChannels = std::make_unique<std::unique_ptr<Point3[]>[]>(_nMaps);
    for (map_channel_size_type iMapChannel = 0;
         iMapChannel < map_channel_count_; ++iMapChannel) {
      _mapChannels[iMapChannel] = std::make_unique<Point3[]>(_nVertices);
    }
  }

  size_type size() const {
    return _nVertices;
  }

  size_type map_channel_size() const {
    return _nMaps;
  }

  const_vertex_attribute_viewer operator[](size_type index_) const {
    return {*this, index_};
  }

  vertex_attribute_viewer operator[](size_type index_) {
    return {*this, index_};
  }

  std::pair<vertex_list, std::unique_ptr<size_type[]>> to_indexed() const {
    vertex_list resultVertexList{_nVertices, _nMaps};
    size_type nResultVertices = 0;
    auto indices = std::make_unique<size_type[]>(_nVertices);
    for (size_type iVertex = 0; iVertex < _nVertices; ++iVertex) {
      auto currentVertex = (*this)[iVertex];
      bool found = false;
      for (size_type jResultVertex = 0; jResultVertex < nResultVertices;
           ++jResultVertex) {
        if (currentVertex == resultVertexList[jResultVertex]) {
          indices[iVertex] = jResultVertex;
          found = true;
          break;
        }
      }
      if (!found) {
        resultVertexList[nResultVertices] = currentVertex;
        indices[iVertex] = nResultVertices;
        ++nResultVertices;
      }
    }
    return {
        resultVertexList._copyFirstNVertices(nResultVertices),
        std::move(indices),
    };
  }

private:
  size_type _nVertices;
  map_channel_size_type _nMaps;
  std::unique_ptr<Point3[]> _vertexChannel;
  std::unique_ptr<Point3[]> _normalChannel;
  std::unique_ptr<Point3[]> _texcoordChannel;
  std::unique_ptr<std::unique_ptr<Point3[]>[]> _mapChannels;

  vertex_list _copyFirstNVertices(size_type n_) {
    vertex_list result{n_, _nMaps};
    std::copy_n(_vertexChannel.get(), n_, result._vertexChannel.get());
    std::copy_n(_normalChannel.get(), n_, result._normalChannel.get());
    std::copy_n(_texcoordChannel.get(), n_, result._texcoordChannel.get());
    for (map_channel_size_type iMap = 0; iMap < _nMaps; ++iMap) {
      std::copy_n(_mapChannels[iMap].get(), n_,
                  result._mapChannels[iMap].get());
    }
    return result;
  }
};

struct export_settings {
  enum class index_type_setting {
    unsigned_8,
    unsigned_16,
    unsigned_32,
  } index_type;

  enum class index_setting {
    fixed,
    at_least,
  } index;
};

template <fx::gltf::Accessor::ComponentType ComponentType>
struct glTf_component_type_traits {};

template <>
struct glTf_component_type_traits<
    fx::gltf::Accessor::ComponentType::UnsignedByte> {
  using storage_type = std::uint8_t;
};

template <>
struct glTf_component_type_traits<
    fx::gltf::Accessor::ComponentType::UnsignedShort> {
  using storage_type = std::uint16_t;
};

template <>
struct glTf_component_type_traits<
    fx::gltf::Accessor::ComponentType::UnsignedInt> {
  using storage_type = std::uint32_t;
};

class glTf_overflow : public std::exception {
public:
  enum class target_type {
    accessor_count,
    buffer_count,
    buffer_view_count,
    mesh_count,
    node_count,
    scene_count,

    buffer,
  };

  glTf_overflow(target_type target_) : _target(target_) {
  }

  target_type target() const {
    return _target;
  }

private:
  target_type _target;
};

class glTf_creator {
public:
  using node_handle = std::uint32_t;

  using mesh_handle = std::uint32_t;

  using scene_handle = std::uint32_t;

  glTf_creator(const export_settings &settings_) : _settings(settings_) {
  }

  scene_handle add_scene() {
    fx::gltf::Scene glTfScene;
    return _addScene(std::move(glTfScene));
  }

  node_handle add_node(INode &max_node_) {
    auto maxNodeName = std::basic_string_view<MCHAR>(max_node_.GetName());
    auto name = win32::mchar_to_utf8(maxNodeName);

    auto nodeTM = max_node_.GetNodeTM(0);
    Matrix3 nodeLocalTM(1);
    if (max_node_.IsRootNode()) {
      nodeLocalTM = nodeTM;
    } else {
      auto parentNodeTM = max_node_.GetParentNode()->GetNodeTM(0);
      nodeLocalTM = nodeTM * Inverse(parentNodeTM);
    }

    auto objectOffsetTM = _getObjectOffsetTM(max_node_);

    auto objectLocalTM = objectOffsetTM * nodeLocalTM;
    auto trans = objectLocalTM.GetTrans();
    Point3 pos;
    Point3 scale;
    Quat rot;
    DecomposeMatrix(objectLocalTM, pos, rot, scale);

    fx::gltf::Node glTfNode;
    glTfNode.name = name;
    if (pos != Point3::Origin) {
      _writePoint3(glTfNode.translation.data(), pos);
    }
    if (scale != Point3(1, 1, 1)) {
      _writePoint3(glTfNode.scale.data(), scale);
    }
    if (!rot.IsIdentity()) {
      rot.Normalize();
      _writeQuat(glTfNode.rotation.data(), rot);
    }

    return _addNode(std::move(glTfNode));
  }

  mesh_handle add_tri_object(TriObject &tri_obj_) {
    auto &mesh = tri_obj_.GetMesh();
    auto maxNodeName = std::basic_string_view<MCHAR>(
        tri_obj_.NodeName(), tri_obj_.NodeName().Length());
    auto name = win32::mchar_to_utf8(maxNodeName);

    const auto nFaces = mesh.getNumFaces();
    const auto nMaps = mesh.getNumMaps();
    const auto nVData = mesh.getNumVData();

    vertex_list vertices(nFaces * 3, 0);

    for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
         ++iFace) {
      auto &face = mesh.faces[iFace];
      for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
        auto iVertex = face.getVert(iFaceVert);
        auto outVertex = vertices[iFace * 3 + iFaceVert];
        auto &vertex = mesh.getVert(iVertex);
        outVertex.vertex(vertex);
        auto &normal = mesh.getNormal(iVertex);
        outVertex.normal(normal);
      }
    }

    if (mesh.getNumTVerts() != 0) {
      for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
           ++iFace) {
        auto &tface = mesh.tvFace[iFace];
        for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
          auto iTVertex = tface.getTVert(iFaceVert);
          auto outVertex = vertices[iFace * 3 + iFaceVert];
          auto &uvw = mesh.getTVert(iTVertex);
          outVertex.texcoord(uvw);
        }
      }
    }

    for (std::remove_const_t<decltype(nMaps)> iMap = 0; iMap < nMaps; ++iMap) {
    }

    for (std::remove_const_t<decltype(nVData)> iVData = 0; iVData < nVData;
         ++iVData) {
      auto vData = mesh.vData[iVData];
    }

    auto [indexedVertices, indices] = vertices.to_indexed();
    return _addMesh(name, indexedVertices,
                    gsl::make_span(indices.get(), vertices.size()));
  }

  void set_default_scene(scene_handle scene_) {
    assert(scene_ < _glTfDocument.scenes.size());
    _glTfDocument.scene = scene_;
  }

  void set_root_node(scene_handle scene_, node_handle node_) {
    assert(scene_ < _glTfDocument.scenes.size());
    assert(node_ < _glTfDocument.nodes.size());
    _glTfDocument.scenes[scene_].nodes.push_back(node_);
  }

  void set_parent(node_handle child_, node_handle parent_) {
    assert(child_ < _glTfDocument.nodes.size());
    assert(parent_ < _glTfDocument.nodes.size());
    _glTfDocument.nodes[parent_].children.push_back(child_);
  }

  void set_mesh(node_handle node_, mesh_handle mesh_) {
    assert(node_ < _glTfDocument.nodes.size());
    assert(mesh_ < _glTfDocument.meshes.size());
    _glTfDocument.nodes[node_].mesh = mesh_;
  }

  void commit() {
    if (_bufferSegments.length() == 0) {
      (void)_bufferSegments.allocate_view(1);
    }
    _glTfDocument.buffers.emplace_back();
    auto &mainBuffer = _glTfDocument.buffers.back();
    mainBuffer.data.resize(_bufferSegments.length());
    _bufferSegments.read_all(
        reinterpret_cast<std::byte *>(mainBuffer.data.data()));
    mainBuffer.byteLength = _bufferSegments.length();
  }

  /// <summary>
  /// Save as .glb format.
  /// </summary>
  /// <param name="path_"></param>
  /// <param name="binary_dir_name_"></param>
  /// <param name="binary_ext_"></param>
  void save(std::string_view path_,
            std::string_view binary_dir_name_,
            std::string_view binary_ext_) {
    for (decltype(_glTfDocument.buffers)::size_type iBuffer = 0;
         iBuffer != _glTfDocument.buffers.size(); ++iBuffer) {
      auto &buffer = _glTfDocument.buffers[iBuffer];
      std::stringstream urlss;
      urlss << u8"./" << binary_dir_name_;
      if (_glTfDocument.buffers.size() != 1) {
        std::array<char, sizeof(iBuffer) * CHAR_BIT> indexChars;
        auto result = std::to_chars(
            indexChars.data(), indexChars.data() + indexChars.size(), iBuffer);
        assert(result.ec == std::errc());
        // TODO: indexString is not guarenteen as UTF8.
        auto indexString =
            std::string_view(indexChars.data(), result.ptr - indexChars.data());
        urlss << u8"-" << indexString;
      }
      urlss << binary_ext_;
      buffer.uri = std::filesystem::u8path(urlss.str()).u8string();
    }
    _doSave(path_, false);
  }

  /// <summary>
  /// Save as .gltf file.
  /// </summary>
  /// <param name="path_"></param>
  void save(std::string_view path_) {
    for (auto &buffer : _glTfDocument.buffers) {
      buffer.uri.clear();
    }
    _doSave(path_, true);
  }

private:
  using _buffer_type = std::vector<std::byte>;

  class _unmerged_buffer_segments {
  public:
    _unmerged_buffer_segments(std::uint32_t index_) : _index(index_) {
    }

    std::pair<fx::gltf::BufferView, std::byte *>
    allocate_view(std::uint32_t size_) {
      fx::gltf::BufferView bufferView;
      bufferView.buffer = _index;
      bufferView.byteOffset = _length;
      bufferView.byteLength = size_;
      bufferView.byteStride = 0;
      _length += size_;
      _buffers.emplace_back(size_);
      return {bufferView, _buffers.back().data()};
    }

    std::uint32_t index() {
      return _index;
    }

    std::uint32_t length() {
      return _length;
    }

    void read_all(std::byte *output_) {
      for (auto &buffer : _buffers) {
        std::copy(buffer.begin(), buffer.end(), output_);
        output_ += buffer.size();
      }
    }

  private:
    std::uint32_t _index;
    std::uint32_t _length = 0;
    std::list<_buffer_type> _buffers;
  };

  _unmerged_buffer_segments _bufferSegments = 0;
  fx::gltf::Document _glTfDocument;
  const export_settings &_settings;

  void _doSave(std::string_view path_, bool binary_) try {
    auto path = std::filesystem::u8path(path_).string();
    fx::gltf::Save(_glTfDocument, path, binary_);
  } catch (const std::exception &exception_) {
    DebugPrint(L"glTf-exporter: exception occured when save:");
    _printException(exception_);
  } catch (...) {
    DebugPrint(L"glTf-exporter: Unknown exception.");
  }

  void _printException(const std::exception &e, int level = 0) {
    std::cerr << std::string(level, ' ') << "exception: " << e.what() << '\n';
    try {
      std::rethrow_if_nested(e);
    } catch (const std::exception &e) {
      _printException(e, level + 1);
    } catch (...) {
    }
  }

  std::uint32_t _addMesh(std::string_view name_,
                         const vertex_list &vertex_list_,
                         gsl::span<vertex_list::size_type> indices_) {
    auto indicesAccessorIndex = _addIndices(indices_);
    fx::gltf::Primitive primitive;
    primitive.mode = fx::gltf::Primitive::Mode::Triangles;
    primitive.indices = indicesAccessorIndex;

    auto szMainVB = 3 * 4 * vertex_list_.size();
    auto [mainVBView, mainVB] = _bufferSegments.allocate_view(szMainVB);
    auto mainVBViewIndex = _addBufferView(std::move(mainVBView));

    fx::gltf::Accessor positionAccessor;
    positionAccessor.bufferView = mainVBViewIndex;
    positionAccessor.type = fx::gltf::Accessor::Type::Vec3;
    positionAccessor.componentType = fx::gltf::Accessor::ComponentType::Float;
    positionAccessor.count = vertex_list_.size();
    positionAccessor.min = {std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max()};
    positionAccessor.max = {std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest()};
    auto &min = positionAccessor.min;
    auto &max = positionAccessor.max;
    for (vertex_list::size_type iVertex = 0; iVertex != vertex_list_.size();
         ++iVertex) {
      auto &v = vertex_list_[iVertex].vertex();
      min[0] = std::min(v.x, min[0]);
      min[1] = std::min(v.y, min[1]);
      min[2] = std::min(v.z, min[2]);
      max[0] = std::max(v.x, max[0]);
      max[1] = std::max(v.y, max[1]);
      max[2] = std::max(v.z, max[2]);
      auto pOutput = reinterpret_cast<float *>(mainVB + 3u * 4u * iVertex);
      pOutput[0] = v.x;
      pOutput[1] = v.y;
      pOutput[2] = v.z;
    }
    auto positionAccessorIndex = _addAccessor(std::move(positionAccessor));
    primitive.attributes.emplace(u8"POSITION", positionAccessorIndex);

    fx::gltf::Mesh mesh;
    mesh.name = name_;
    mesh.primitives = {std::move(primitive)};
    return _addMesh(std::move(mesh));
  }

  std::uint32_t _addIndices(gsl::span<vertex_list::size_type> indices_) {
    static_assert(export_settings::index_type_setting::unsigned_8 ==
                  static_cast<export_settings::index_type_setting>(0));
    static_assert(export_settings::index_type_setting::unsigned_16 ==
                  static_cast<export_settings::index_type_setting>(1));
    static_assert(export_settings::index_type_setting::unsigned_32 ==
                  static_cast<export_settings::index_type_setting>(2));

    static const std::array<decltype(indices_.size()), 3> capacitys = {
        std::numeric_limits<std::uint8_t>::max(),
        std::numeric_limits<std::uint16_t>::max(),
        std::numeric_limits<std::uint32_t>::max()};

    static auto getCapacityIndex =
        [](export_settings::index_type_setting type_) {
          return static_cast<
              std::underlying_type_t<export_settings::index_type_setting>>(
              type_);
        };

    static auto getTypeFromCapacityIndex =
        [](decltype(capacitys.size()) index_) {
          return static_cast<export_settings::index_type_setting>(index_);
        };

    std::optional<export_settings::index_type_setting> usingType;
    if (_settings.index == export_settings::index_setting::fixed) {
      if (indices_.size() <=
          capacitys[getCapacityIndex(_settings.index_type)]) {
        usingType = _settings.index_type;
      }
    } else {
      for (auto iCap = getCapacityIndex(_settings.index_type);
           iCap < capacitys.size(); ++iCap) {
        auto cap = capacitys[iCap];
        if (cap >= indices_.size()) {
          usingType = getTypeFromCapacityIndex(iCap);
          break;
        }
      }
    }

    if (!usingType) {
      throw std::runtime_error("Index too large.");
    }

    assert(indices_.size() <= capacitys[getCapacityIndex(*usingType)]);

    std::uint32_t sizeofIndex = 0;
    switch (*usingType) {
    case export_settings::index_type_setting::unsigned_8: {
      sizeofIndex = 1;
      break;
    }
    case export_settings::index_type_setting::unsigned_16: {
      sizeofIndex = 2;
      break;
    }
    case export_settings::index_type_setting::unsigned_32: {
      sizeofIndex = 4;
      break;
    }
    }

    auto [bufferView, bufferViewData] =
        _bufferSegments.allocate_view(_ensureNotOverflow<std::uint32_t>(
            glTf_overflow::target_type::buffer, sizeofIndex * indices_.size()));

    auto glTfComponentType = fx::gltf::Accessor::ComponentType::None;
    switch (*usingType) {
    case export_settings::index_type_setting::unsigned_8: {
      glTfComponentType = fx::gltf::Accessor::ComponentType::UnsignedByte;
      std::copy_n(indices_.data(), indices_.size(),
                  reinterpret_cast<std::uint8_t *>(bufferViewData));
      break;
    }
    case export_settings::index_type_setting::unsigned_16: {
      glTfComponentType = fx::gltf::Accessor::ComponentType::UnsignedShort;
      std::copy_n(indices_.data(), indices_.size(),
                  reinterpret_cast<std::uint16_t *>(bufferViewData));
      break;
    }
    case export_settings::index_type_setting::unsigned_32: {
      glTfComponentType = fx::gltf::Accessor::ComponentType::UnsignedInt;
      std::copy_n(indices_.data(), indices_.size(),
                  reinterpret_cast<std::uint32_t *>(bufferViewData));
      break;
    }
    }

    bufferView.target = fx::gltf::BufferView::TargetType::ElementArrayBuffer;
    auto bufferViewIndex = _addBufferView(std::move(bufferView));

    fx::gltf::Accessor accessor;
    accessor.componentType = glTfComponentType;
    accessor.count = static_cast<std::uint32_t>(indices_.size());
    accessor.type = fx::gltf::Accessor::Type::Scalar;
    accessor.bufferView = bufferViewIndex;

    return _addAccessor(std::move(accessor));
  }

  std::uint32_t _addScene(fx::gltf::Scene &&scene_) {
    auto result = _glTfDocument.scenes.size();
    _glTfDocument.scenes.push_back(std::move(scene_));
    return _ensureAssetCount(glTf_overflow::target_type::scene_count, result);
  }

  std::uint32_t _addNode(fx::gltf::Node &&node_) {
    auto result = _glTfDocument.nodes.size();
    _glTfDocument.nodes.push_back(std::move(node_));
    return _ensureAssetCount(glTf_overflow::target_type::node_count, result);
  }

  std::uint32_t _addMesh(fx::gltf::Mesh &&mesh_) {
    auto result = _glTfDocument.meshes.size();
    _glTfDocument.meshes.push_back(std::move(mesh_));
    return _ensureAssetCount(glTf_overflow::target_type::mesh_count, result);
  }

  std::uint32_t _addBufferView(fx::gltf::BufferView &&buffer_view_) {
    auto result = _glTfDocument.bufferViews.size();
    _glTfDocument.bufferViews.push_back(std::move(buffer_view_));
    return _ensureAssetCount(glTf_overflow::target_type::buffer_view_count,
                             result);
  }

  std::uint32_t _addAccessor(fx::gltf::Accessor &&accessor_) {
    auto result = _glTfDocument.accessors.size();
    _glTfDocument.accessors.push_back(std::move(accessor_));
    return _ensureAssetCount(glTf_overflow::target_type::accessor_count,
                             result);
  }

  template <typename UInt>
  std::uint32_t _ensureAssetCount(glTf_overflow::target_type target_,
                                  UInt count_) {
    return _ensureNotOverflow<std::uint32_t>(target_, count_);
  }

  template <typename CapacityInt, typename UInt>
  std::uint32_t _ensureNotOverflow(glTf_overflow::target_type target_,
                                   UInt count_) {
    if (count_ > std::numeric_limits<CapacityInt>::max()) {
      throw glTf_overflow(target_);
    }
    return static_cast<CapacityInt>(count_);
  }

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
};

class main_visitor : public ITreeEnumProc {
public:
  main_visitor(glTf_creator &creator_)
      : _creator(creator_), _sceneHandle(_creator.add_scene()) {
    _creator.set_default_scene(_sceneHandle);
  }

  int callback(INode *max_node_) override {
    auto glTfNode = _creator.add_node(*max_node_);
    if (!max_node_->IsRootNode()) {
      auto parentMaxNode = max_node_->GetParentNode();
      auto rParentGlTfNode = _nodeMaps.find(parentMaxNode);
      if (rParentGlTfNode != _nodeMaps.end()) {
        _creator.set_parent(glTfNode, rParentGlTfNode->second);
      } else {
        _creator.set_root_node(_sceneHandle, glTfNode);
      }
    }
    auto object = max_node_->EvalWorldState(0).obj;
    if (object->CanConvertToType({TRIOBJ_CLASS_ID, 0})) {
      auto triObject = static_cast<TriObject *>(
          object->ConvertToType(0, {TRIOBJ_CLASS_ID, 0}));
      if (triObject) {
        auto glTfMesh = _creator.add_tri_object(*triObject);
        _creator.set_mesh(glTfNode, glTfMesh);
        if (triObject != object) {
          triObject->DeleteMe();
        }
      }
    }
    _nodeMaps.emplace(max_node_, glTfNode);
    return TREE_CONTINUE;
  }

private:
  glTf_creator &_creator;
  std::unordered_map<INode *, glTf_creator::node_handle> _nodeMaps;
  glTf_creator::scene_handle _sceneHandle;
};

void *glTf_exporter::class_description::Create(BOOL) {
  return new glTf_exporter();
}

const MCHAR *glTf_exporter::class_description::ClassName() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_CLASS_NAME);
}

Class_ID glTf_exporter::class_description::ClassID() {
  static Class_ID id = Class_ID(0x4b4b76a0, 0x68434828);
  return id;
}

const MCHAR *glTf_exporter::class_description::Category() {
  return _M("");
}

const MCHAR *glTf_exporter::class_description::InternalName() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_INTERNAL_NAME);
}

HINSTANCE glTf_exporter::class_description::HInstance() {
  return win32::get_instance();
}

int glTf_exporter::ExtCount() {
  return static_cast<int>(glTf_extension_info_list.size());
}

const MCHAR *glTf_exporter::Ext(int i_) {
  if (i_ < 0 || i_ >= glTf_extension_info_list.size()) {
    return _M("");
  } else {
    return glTf_extension_info_list[i_].rep;
  }
}

const MCHAR *glTf_exporter::LongDesc() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_LONG_DESCRIPTION);
}

const MCHAR *glTf_exporter::ShortDesc() {
  auto result = win32::get_string_resource(IDS_GLTF_EXPORTER_SHORT_DESCRIPTION);
  return result;
}

const MCHAR *glTf_exporter::AuthorName() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_AUTHOR_NAME);
}

const MCHAR *glTf_exporter::CopyrightMessage() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_COPYRIGHT_MESSAGE);
}

const MCHAR *glTf_exporter::OtherMessage1() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_OTHER_MESSAGE_1);
}

const MCHAR *glTf_exporter::OtherMessage2() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_OTHER_MESSAGE_2);
}

unsigned int glTf_exporter::Version() {
  return 100;
}

void glTf_exporter::ShowAbout(HWND hWnd) {
  MessageBox(hWnd, win32::get_string_resource(IDS_GLTF_EXPORTER_ABOUT),
             _M("About"), MB_OK);
}

int glTf_exporter::DoExport(const MCHAR *name,
                            ExpInterface *ei,
                            Interface *i,
                            BOOL suppressPrompts,
                            DWORD options) {
  auto path = std::filesystem::path(name);

  export_settings settings;
  settings.index = export_settings::index_setting::at_least;
  settings.index_type = export_settings::index_type_setting::unsigned_8;

  glTf_creator creator{settings};

  main_visitor visitor{creator};
  ei->theScene->EnumTree(&visitor);
  creator.commit();

  auto extStr = path.extension().string();
  auto extStrLower = extStr;
  std::transform(extStrLower.begin(), extStrLower.end(), extStrLower.begin(),
                 ::tolower);
  auto u8Name = path.u8string();
  if (extStrLower == ".glb") {
    creator.save(u8Name);
  } else {
    creator.save(u8Name, path.stem().u8string(), u8".BIN");
  }
  return 1;
}

BOOL glTf_exporter::SupportsOptions(int ext, DWORD options) {
  return 1;
}
} // namespace tapu