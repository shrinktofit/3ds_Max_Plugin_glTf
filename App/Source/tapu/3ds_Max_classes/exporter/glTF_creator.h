
#pragma once

#include <Max.h>
#include <cstdint>
#include <fx/gltf.h>
#include <gsl/span>
#include <locale>
#include <string>
#include <string_view>
#include <tapu/3ds_Max_classes/exporter/export_settings.h>
#include <tapu/3ds_Max_classes/exporter/vertex_list.h>
#include <tapu/support/win32/mchar_to_utf8.h>

std::string to_glTF_string(std::u8string_view string_) {
  std::locale locale(std::locale::empty(), new std::codecvt<char32_t, char, std::mbstate_t>());
  return std::string(reinterpret_cast<const char *>(string_.data()),
                     string_.size());
}

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

template <class T> struct dependent_false : std::false_type {};

bool undo_parents_offset(INode &node_, Point3 &point_, Quat &offset_rotation_) {
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

Matrix3 get_local_node_tm(INode &max_node_, TimeValue time_) {
  auto worldTM = max_node_.GetNodeTM(time_);
  if (!max_node_.GetParentNode()->IsRootNode()) {
    auto inverseParent = Inverse(max_node_.GetParentNode()->GetNodeTM(time_));
    return worldTM * inverseParent;
  } else {
    return worldTM;
  }
}

namespace tapu {
class glTf_creator {
public:
  using node_handle = std::uint32_t;

  using mesh_handle = std::uint32_t;

  using animation_handle = std::uint32_t;

  using scene_handle = std::uint32_t;

  glTf_creator(const export_settings &settings_, Interface &max_interface_)
      : _settings(settings_), _maxInterface(max_interface_) {
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

  void add_node_animation(INode &max_node_, node_handle node_) {
    auto transformController = max_node_.GetTMController();
    auto positionTrack = _readPositionTrack(max_node_, *transformController);
    auto scaleTrack = _readScaleTrack(max_node_, *transformController);
    auto rotationTrack = _readRotationTrack(max_node_, *transformController);
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
      (void)_bufferSegments.allocate_view(1, 1);
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
  void save(std::u8string_view path_,
            std::u8string_view binary_dir_name_,
            std::u8string_view binary_ext_) {
    for (decltype(_glTfDocument.buffers)::size_type iBuffer = 0;
         iBuffer != _glTfDocument.buffers.size(); ++iBuffer) {
      auto &buffer = _glTfDocument.buffers[iBuffer];
      std::basic_stringstream<char8_t> urlss;
      urlss << u8"./" << binary_dir_name_;
      if (_glTfDocument.buffers.size() != 1) {
        std::array<char, sizeof(iBuffer) * CHAR_BIT> indexChars;
        auto result = std::to_chars(
            indexChars.data(), indexChars.data() + indexChars.size(), iBuffer);
        assert(result.ec == std::errc());
        // TODO: indexString is not guarenteen as UTF8.
        auto indexString = std::u8string_view(
            reinterpret_cast<const char8_t *>(indexChars.data()),
            result.ptr - indexChars.data());
        urlss << u8"-" << indexString;
      }
      urlss << binary_ext_;
      buffer.uri =
          to_glTF_string(std::filesystem::path(urlss.str()).u8string());
    }
    _doSave(path_, false);
  }

  /// <summary>
  /// Save as .gltf file.
  /// </summary>
  /// <param name="path_"></param>
  void save(std::u8string_view path_) {
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
    allocate_view(std::uint32_t size_, std::uint32_t alignment_) {
      if (alignment_ != 0) {
        if (auto remainder = _length % alignment_; remainder != 0) {
          auto complement = alignment_ - remainder;
          _length += complement;
          _buffers.emplace_back(complement);
        }
      }
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
  Interface &_maxInterface;

  void _doSave(std::u8string_view path_, bool binary_) try {
    auto path = std::filesystem::path(path_).string();
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
    auto [mainVBView, mainVB] = _bufferSegments.allocate_view(szMainVB, 4);
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
      auto pOutput = reinterpret_cast<float *>(
          mainVB + static_cast<std::size_t>(3u * 4u * iVertex));
      pOutput[0] = v.x;
      pOutput[1] = v.y;
      pOutput[2] = v.z;
    }
    auto positionAccessorIndex = _addAccessor(std::move(positionAccessor));
    primitive.attributes.emplace(to_glTF_string(u8"POSITION"),
                                 positionAccessorIndex);

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

    auto [bufferView, bufferViewData] = _bufferSegments.allocate_view(
        _ensureNotOverflow<std::uint32_t>(glTf_overflow::target_type::buffer,
                                          sizeofIndex * indices_.size()),
        sizeofIndex);

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

  enum class _track_kind {
    translation,
    scale,
    rotation,
  };

  template <_track_kind TrackKind> struct _track_value {};

  template <> struct _track_value<_track_kind::translation> {
    using type = Point3;
  };

  template <> struct _track_value<_track_kind::scale> { using type = Point3; };

  template <> struct _track_value<_track_kind::rotation> { using type = Quat; };

  template <_track_kind TrackKind>
  using _track_value_t = typename _track_value<TrackKind>::type;

  template <_track_kind TrackKind> class _xx_track {
    using value_type = _track_value_t<TrackKind>;

  public:
    _xx_track(std::uint32_t size_) : _times(size_), _values(size_) {
    }

    void set(std::uint32_t index_, TimeValue time_, const value_type &value_) {
      _times[index_] = time_;
      _values[index_] = value_;
    }

    TimeValue &timeAt(std::uint32_t index_) {
      return _times[index_];
    }

    value_type &valueAt(std::uint32_t index_) {
      return _values[index_];
    }

    _xx_track subtrack_pre(std::uint32_t size_) {
      assert(size_ <= _values.size());
      _xx_track result(size_);
      std::copy_n(_times.begin(), size_, result._times.begin());
      std::copy_n(_values.begin(), size_, result._values.begin());
      return result;
    }

  private:
    std::vector<TimeValue> _times;
    std::vector<value_type> _values;
  };

  _xx_track<_track_kind::translation>
  _readPositionTrack(INode &max_node_, Control &transform_control_) {
    return _readTRSTrack<_track_kind::translation>(
        max_node_, transform_control_.GetPositionController());
  }

  _xx_track<_track_kind::scale> _readScaleTrack(INode &max_node_,
                                                Control &transform_control_) {
    return _readTRSTrack<_track_kind::scale>(
        max_node_, transform_control_.GetScaleController());
  }

  _xx_track<_track_kind::rotation>
  _readRotationTrack(INode &max_node_, Control &transform_control_) {
    return _readTRSTrack<_track_kind::rotation>(
        max_node_, transform_control_.GetRotationController());
  }

  template <_track_kind TrackKind>
  _xx_track<TrackKind> _readTRSTrack(INode &max_node_, Control *control_) {
    auto classId =
        TrackKind == _track_kind::translation
            ? TCBINTERP_POSITION_CLASS_ID
            : (TrackKind == _track_kind::scale ? TCBINTERP_SCALE_CLASS_ID
                                               : TCBINTERP_ROTATION_CLASS_ID);

    if (control_ && control_->NumKeys() != NOT_KEYFRAMEABLE &&
        control_->ClassID() == Class_ID(classId, 0)) {
      auto keyControl =
          reinterpret_cast<IKeyControl *>(control_->GetInterface(I_KEYCONTROL));
      assert(keyControl);
      return _readTCBKeys<TrackKind>(max_node_, *keyControl);
    } else {
      return _readLinearKeys<TrackKind>(max_node_);
    }
  }

  template <_track_kind TrackKind>
  _xx_track<TrackKind> _readTCBKeys(INode &max_node_,
                                    IKeyControl &key_control_) {
    auto nKeys = key_control_.GetNumKeys();
    _xx_track<TrackKind> track(nKeys);
    for (decltype(nKeys) iKey = 0; iKey != nKeys; ++iKey) {
      _track_value_t<TrackKind> trackValue;
      TimeValue time;
      if constexpr (TrackKind == _track_kind::translation) {
        ITCBPoint3Key key;
        key_control_.GetKey(iKey, &key);
        trackValue = key.val;
        time = key.time;
      } else if constexpr (TrackKind == _track_kind::scale) {
        ITCBScaleKey key;
        key_control_.GetKey(iKey, &key);
        trackValue = key.val.s;
        time = key.time;
      } else if constexpr (TrackKind == _track_kind::rotation) {
        ITCBRotKey key;
        key_control_.GetKey(iKey, &key);
        auto qKey = QFromAngAxis(key.val.angle, key.val.axis);
        trackValue = qKey;
        time = key.time;
      } else {
        static_assert(dependent_false<decltype(ClassId)>::value,
                      "Non-exhaused.");
      }
      _postprocessTrackValue<TrackKind>(max_node_, trackValue);
      track.set(iKey, time / GetTicksPerFrame(), trackValue);
    }
    return track;
  }

  template <_track_kind TrackKind>
  _xx_track<TrackKind> _readLinearKeys(INode &max_node_) {
    const auto ticksPerFrame = GetTicksPerFrame();
    auto animRange = _maxInterface.GetAnimRange();
    auto nEssentialKeys = (animRange.End() - animRange.Start()) / ticksPerFrame;
    ++nEssentialKeys;

    _xx_track<TrackKind> track(nEssentialKeys);
    std::uint32_t nKey = 0;
    auto time = animRange.Start();
    _track_value_t<TrackKind> rate;

    for (decltype(nEssentialKeys) iEssentialKey = 0;
         iEssentialKey < nEssentialKeys;
         ++iEssentialKey, time += ticksPerFrame) {
      auto localTM = get_local_node_tm(max_node_, time);
      AffineParts affineParts;
      decomp_affine(localTM, &affineParts);

      _track_value_t<TrackKind> trackValue;
      if constexpr (TrackKind == _track_kind::translation) {
        trackValue = affineParts.t;
      } else if constexpr (TrackKind == _track_kind::scale) {
        trackValue = ScaleValue(affineParts.k, affineParts.u).s;
        if (affineParts.f < 0) {
          trackValue = -trackValue;
        }
      } else if constexpr (TrackKind == _track_kind::rotation) {
        trackValue = affineParts.q;
      } else {
        static_assert(dependent_false<decltype(ClassId)>::value,
                      "Non-exhaused.");
      }

      if (iEssentialKey == 0 ||
          !_approxEqual(trackValue, track.valueAt(nKey - 1))) {
        track.set(nKey++, time, trackValue);
      }
    }
    auto reducedTrack = track.subtrack_pre(nKey);
    return track;
  }

  template <_track_kind TrackKind>
  void _postprocessTrackValue(INode &max_node_,
                              _track_value_t<TrackKind> &value_) {
    if constexpr (TrackKind == _track_kind::translation) {
      Quat q;
      undo_parents_offset(max_node_, value_, q);
    } else if constexpr (TrackKind == _track_kind::scale) {
      // Do nothing.
    } else if constexpr (TrackKind == _track_kind::rotation) {
      auto qOffset = value_ / Inverse(max_node_.GetObjOffsetRot());
      Point3 p;
      undo_parents_offset(max_node_, p, qOffset);
      value_ = qOffset;
    } else {
      static_assert(dependent_false<decltype(ClassId)>::value, "Non-exhaused.");
    }
  }

  template <_track_kind TrackKind>
  std::uint32_t _writeTrack(const _xx_track<TrackKind> &track_) {
    return 0;
  }

  template <_track_kind TrackKind> std::string _getChannelPath() {
    if constexpr (TrackKind == _track_kind::translation) {
      return u8"translation";
    } else if constexpr (TrackKind == _track_kind::scale) {
      return u8"scale";
    } else if constexpr (TrackKind == _track_kind::rotation) {
      return u8"rotation";
    } else {
      static_assert(dependent_false<decltype(ClassId)>::value, "Non-exhaused.");
    }
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
};
} // namespace tapu