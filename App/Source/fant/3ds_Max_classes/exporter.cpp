
#include "resource.h"
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

enum class attribute_semantic {
  position,
  normal,
  texcoord,
  color,
};

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

class main_visitor : public ITreeEnumProc {
public:
  main_visitor(Interface &max_interface_,
               glTF::document &document_,
               const export_settings &settings_)
      : _maxInterface(max_interface_), _document(document_),
        _glTFScene(document_.make<glTF::scene>()), _settings(settings_) {
    _document.default_scene(_glTFScene);
  }

  int callback(INode *max_node_) override {
    auto glTFNode = _convertNode(*max_node_);

    if (!max_node_->IsRootNode()) {
      auto parentMaxNode = max_node_->GetParentNode();
      auto rParentGlTfNode = _nodeMaps.find(parentMaxNode);
      if (rParentGlTfNode != _nodeMaps.end()) {
        rParentGlTfNode->second->add_child(glTFNode);
      } else {
        _glTFScene->add_node(glTFNode);
      }
    }

    auto object = max_node_->EvalWorldState(0).obj;
    if (object->CanConvertToType({TRIOBJ_CLASS_ID, 0})) {
      auto triObject = static_cast<TriObject *>(
          object->ConvertToType(0, {TRIOBJ_CLASS_ID, 0}));
      if (triObject) {
        auto glTFMesh = _convertTriObj(*triObject);
        glTFNode->mesh(glTFMesh);
        if (triObject != object) {
          triObject->DeleteMe();
        }
      }
    }

    auto objectRef = max_node_->GetObjectRef();
    if (objectRef->SuperClassID() == GEN_DERIVOB_CLASS_ID) {
      auto derivedObject = reinterpret_cast<IDerivedObject *>(objectRef);
      const auto numModifiers = derivedObject->NumModifiers();
      for (std::remove_const_t<decltype(numModifiers)> iModifier = 0;
           iModifier < numModifiers; ++iModifier) {
        auto modifier = derivedObject->GetModifier(iModifier);
        auto skin = reinterpret_cast<ISkin *>(modifier->GetInterface(I_SKIN));
        if (!skin) {
          continue;
        }

        auto skinContextData = skin->GetContextInterface(max_node_);
        if (!skinContextData) {
          continue;
        }

        const auto numPoints = skinContextData->GetNumPoints();
        for (std::remove_const_t<decltype(numPoints)> iPoint = 0;
             iPoint < numPoints; ++iPoint) {
          const auto numAffectBones =
              skinContextData->GetNumAssignedBones(iPoint);
          for (std::remove_const_t<decltype(numAffectBones)> iAffectBone = 0;
               iAffectBone != numAffectBones; ++iAffectBone) {
            auto affectBoneIndex =
                skinContextData->GetAssignedBone(iPoint, iAffectBone);
            auto weight = skinContextData->GetBoneWeight(iPoint, iAffectBone);
          }
        }

        const auto numBones = skin->GetNumBones();
        for (std::remove_const_t<decltype(numBones)> iBone = 0;
             iBone < numBones; ++iBone) {
          auto boneProperty = skin->GetBoneProperty(iBone);
        }
      }
    }

    _nodeMaps.emplace(max_node_, glTFNode);
    _readAnimation(*max_node_);
    return TREE_CONTINUE;
  }

private:
  Interface &_maxInterface;
  glTF::document &_document;
  const export_settings &_settings;
  std::unordered_map<INode *, glTF::object_ptr<glTF::node>> _nodeMaps;
  glTF::object_ptr<glTF::scene> _glTFScene;
  glTF::object_ptr<glTF::buffer> _mainBuffer;

  glTF::object_ptr<glTF::node> _convertNode(INode &max_node_) {
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

    auto glTFNode = _document.make<glTF::node>();
    glTFNode->name(name);

    if (pos != Point3::Origin) {
      glTFNode->set_position(pos.x, pos.y, pos.z);
    }

    if (scale != Point3(1, 1, 1)) {
      glTFNode->set_scale(scale.x, scale.y, scale.z);
    }

    if (!rot.IsIdentity()) {
      rot.Normalize();
      glTFNode->set_rotation(rot.x, rot.y, rot.z, rot.w);
    }

    return glTFNode;
  }

  glTF::object_ptr<glTF::mesh> _convertTriObj(TriObject &tri_obj_) {
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

  glTF::object_ptr<glTF::animation> _readAnimation(INode &max_node_) {
    auto transformController = max_node_.GetTMController();
    auto positionTrack = _readPositionTrack(max_node_, *transformController);
    auto scaleTrack = _readScaleTrack(max_node_, *transformController);
    auto rotationTrack = _readRotationTrack(max_node_, *transformController);
    return _document.make<glTF::animation>();
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

  glTF::object_ptr<glTF::mesh>
  _addMesh(std::u8string_view name_,
           const vertex_list &vertex_list_,
           gsl::span<vertex_list::size_type> indices_) {
    auto indicesAccessor = _addIndices(indices_);

    auto szMainVB = 3 * 4 * vertex_list_.size();
    auto mainVBView =
        _document.make<glTF::buffer_view>(_mainBuffer, szMainVB, 4);

    auto positionAccessor = _document.make<glTF::accessor>(
        mainVBView, 0, glTF::accessor::component_type::the_float,
        glTF::accessor::type_type::vec3, vertex_list_.size());
    for (vertex_list::size_type iVertex = 0; iVertex != vertex_list_.size();
         ++iVertex) {
      auto &v = vertex_list_[iVertex].vertex();
      auto pOutput = reinterpret_cast<float *>(
          mainVBView->data() + static_cast<std::size_t>(3u * 4u * iVertex));
      pOutput[0] = v.x;
      pOutput[1] = v.y;
      pOutput[2] = v.z;
    }

    glTF::primitive primitive{glTF::primitive::mode_type::triangles};
    primitive.indices(indicesAccessor);
    primitive.emplace_attribute(glTF::standard_semantics::position,
                                positionAccessor);

    auto glTFMesh = _document.make<glTF::mesh>();
    glTFMesh->name(name_);
    glTFMesh->push_primitive(primitive);

    return glTFMesh;
  }

  glTF::object_ptr<glTF::accessor>
  _addIndices(gsl::span<vertex_list::size_type> indices_) {
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

    auto bufferView = _document.make<glTF::buffer_view>(
        _mainBuffer, static_cast<std::uint32_t>(sizeofIndex * indices_.size()),
        sizeofIndex);
    bufferView->target(glTF::buffer_view::target_type::element_array_buffer);

    auto glTfComponentType = glTF::accessor::component_type::unsigned_byte;
    switch (*usingType) {
    case export_settings::index_type_setting::unsigned_8: {
      glTfComponentType = glTF::accessor::component_type::unsigned_byte;
      std::copy_n(indices_.data(), indices_.size(),
                  reinterpret_cast<std::uint8_t *>(bufferView->data()));
      break;
    }
    case export_settings::index_type_setting::unsigned_16: {
      glTfComponentType = glTF::accessor::component_type::unsigned_short;
      std::copy_n(indices_.data(), indices_.size(),
                  reinterpret_cast<std::uint16_t *>(bufferView->data()));
      break;
    }
    case export_settings::index_type_setting::unsigned_32: {
      glTfComponentType = glTF::accessor::component_type::unsigned_int;
      std::copy_n(indices_.data(), indices_.size(),
                  reinterpret_cast<std::uint32_t *>(bufferView->data()));
      break;
    }
    }

    auto accessor = _document.make<glTF::accessor>(
        bufferView, 0, glTfComponentType, glTF::accessor::type_type::scalar,
        static_cast<std::uint32_t>(indices_.size()));
    return accessor;
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

  glTF::document glTFDocument;
  main_visitor visitor{*i, glTFDocument, settings};
  ei->theScene->EnumTree(&visitor);
  if (glTFDocument.get_size<glTF::buffer>() == 0) {
    auto demandBuffer = glTFDocument.make<glTF::buffer>();
    demandBuffer->allocate(1, 0);
  }

  auto extStr = path.extension().string();
  auto extStrLower = extStr;
  std::transform(extStrLower.begin(), extStrLower.end(), extStrLower.begin(),
                 ::tolower);
  auto u8Name = path.u8string();
  bool isGlb = extStrLower == ".glb";
  auto glTFJsonStr = glTFDocument.serialize(isGlb).dump(4);
  if (isGlb) {
    std::vector<glTF::chunk> chunks;
    chunks.reserve(2);

    // The json chunk.
    chunks.emplace_back(glTF::make_json_chunk(glTFJsonStr));

    // If the first buffer has no uri, it's treated as a chunk.
    std::optional<std::vector<std::byte>> firstBufferData;
    if (glTFDocument.get_size<glTF::buffer>() != 0) {
      auto firstBuffer = glTFDocument.get<glTF::buffer>(0);
      if (!firstBuffer->uri()) {
        firstBufferData.emplace(firstBuffer->size());
        firstBuffer->read_all(firstBufferData->data());
      }
    }
    if (firstBufferData) {
      chunks.emplace_back(glTF::make_buffer_chunk(
          gsl::make_span(firstBufferData->data(), firstBufferData->size())));
    }

    auto glb = glTF::write_glb(chunks.begin(), chunks.end());
    std::basic_ofstream<std::byte> ofs(path, std::ios::binary);
    ofs.write(glb.data(), glb.size());
    ofs.close();
  } else {
    std::ofstream ofs(path);
    ofs << glTFJsonStr;
    ofs.close();
  }

  // Write each buffer specified uri.
  auto nBuffers = glTFDocument.get_size<glTF::buffer>();
  for (decltype(nBuffers) iBuffer = 0; iBuffer < nBuffers; ++iBuffer) {
    auto buffer = glTFDocument.get<glTF::buffer>(iBuffer);
    if (auto uri = buffer->uri(); uri) {
      auto uriPath = std::filesystem::path(*uri);
      if (uriPath.is_relative()) {
        uriPath = path.parent_path() / uriPath;
      }
      std::basic_ofstream<std::byte> ofs(uriPath, std::ios::binary);
      std::vector<std::byte> data(buffer->size());
      buffer->read_all(data.data());
      ofs.write(data.data(), data.size());
    }
  }

  return 1;
}

BOOL glTf_exporter::SupportsOptions(int ext, DWORD options) {
  return 1;
}
} // namespace fant
