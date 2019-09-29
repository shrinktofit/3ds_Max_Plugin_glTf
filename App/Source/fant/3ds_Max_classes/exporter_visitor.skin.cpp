
#include <fant/3ds_Max_classes/exporter_visitor.h>

namespace fant {
glTF::object_ptr<glTF::skin>
exporter_visitor::_tryExportSkin(INode &max_node_, _immediate_mesh &imm_mesh_) {
  auto objectRef = max_node_.GetObjectRef();
  if (objectRef->SuperClassID() != GEN_DERIVOB_CLASS_ID) {
    return nullptr;
  }

  auto derivedObject = reinterpret_cast<IDerivedObject *>(objectRef);
  const auto numModifiers = derivedObject->NumModifiers();
  glTF::object_ptr<glTF::skin> glTFSkin;
  for (std::remove_const_t<decltype(numModifiers)> iModifier = 0;
       iModifier < numModifiers; ++iModifier) {
    auto modifier = derivedObject->GetModifier(iModifier);
    auto skin = reinterpret_cast<ISkin *>(modifier->GetInterface(I_SKIN));
    if (!skin) {
      continue;
    }

    if (skin && glTFSkin) {
      // TODO warning
      continue;
    }

    auto skinContextData = skin->GetContextInterface(&max_node_);
    if (!skinContextData) {
      continue;
    }

    glTFSkin = _convertSkin(max_node_, *skin);

    const auto nPoints = skinContextData->GetNumPoints();
    if (nPoints != imm_mesh_.vertices.vertex_count()) {
      // TODO warning
      assert(false);
      continue;
    }

    bool exceedMaxAllowedJoints = false;
    std::vector<std::pair<std::byte *, std::byte *>> sets;
    auto addset = [&]() {
      auto iset = sets.size();
      auto jointsChannelData = imm_mesh_.vertices.add_channel(
          glTF::standard_semantics::joints(iset),
          glTF::accessor::type_type::vec4,
          glTF::accessor::component_type::unsigned_short);
      auto weightsChannelData = imm_mesh_.vertices.add_channel(
          glTF::standard_semantics::weights(iset),
          glTF::accessor::type_type::vec4,
          glTF::accessor::component_type::the_float);
      sets.push_back({jointsChannelData, weightsChannelData});
    };

    addset();
    auto ensureSets =
        [&](_immediate_mesh::vertex_list::vertex_count_type influence_size_) {
          if (influence_size_ > 4) {
            auto setsreq = influence_size_ / 4;
            if (influence_size_ % 4 != 0) {
              ++setsreq;
            }
            if (sets.size() < setsreq) {
              for (decltype(setsreq) i = 0; i < (setsreq - sets.size()); ++i) {
                addset();
              }
            }
          }
        };

    for (std::remove_const_t<decltype(nPoints)> iPoint = 0; iPoint < nPoints;
         ++iPoint) {
      auto nAssignedBone = skinContextData->GetNumAssignedBones(iPoint);
      ensureSets(nAssignedBone);

      for (std::remove_const_t<decltype(nAssignedBone)> iAssignedBone = 0;
           iAssignedBone != nAssignedBone; ++iAssignedBone) {

        auto iset = iAssignedBone / 4;
        auto iinflu = iAssignedBone % 4;

        auto assignedBoneId =
            skinContextData->GetAssignedBone(iPoint, iAssignedBone);

        using JointStorage = glTF::accessor::component_storage_t<
            glTF::accessor::component_type::unsigned_short>;

        using WeightStorage = glTF::accessor::component_storage_t<
            glTF::accessor::component_type::the_float>;

        reinterpret_cast<JointStorage *>(
            sets[iset].first)[4 * iPoint + iinflu] = assignedBoneId;

        auto weight = skinContextData->GetBoneWeight(iPoint, iAssignedBone);
        reinterpret_cast<WeightStorage *>(
            sets[iset].second)[4 * iPoint + iinflu] = weight;
      }
    }
    if (exceedMaxAllowedJoints) {
      // TODO warning
    }
  }

  return glTFSkin;
}

glTF::object_ptr<glTF::skin> exporter_visitor::_convertSkin(INode &max_node_,
                                                            ISkin &max_skin_) {
  auto glTFSkin = _document.factory().make<glTF::skin>();

  auto skinHostNodeWorldTM = max_node_.GetObjectTM(0);

  const auto nBones = max_skin_.GetNumBones();
  std::vector<Matrix3> inverseBindMatrices(nBones);
  for (std::remove_const_t<decltype(nBones)> iBone = 0; iBone < nBones;
       ++iBone) {
    auto boneProperty = max_skin_.GetBoneProperty(iBone);
    auto boneNode = max_skin_.GetBone(iBone);
    auto rglTFBoneNode = _nodeMaps.find(boneNode);
    if (rglTFBoneNode == _nodeMaps.end()) {
      throw std::runtime_error("Bone node is not in the scene graph.");
    }
    auto glTFBoneNode = rglTFBoneNode->second;
    auto inverseBindMatrix =
        skinHostNodeWorldTM * Inverse(boneNode->GetNodeTM(0));
    inverseBindMatrices[iBone] = _convertIntoGlTFAxisSystem(inverseBindMatrix);
    glTFSkin->add_joint(glTFBoneNode);
  }

  auto inverseBindMatricesAccessor = _makeSimpleAccessor(
      glTF::accessor::type_type::mat4,
      glTF::accessor::component_type::the_float, inverseBindMatrices.size());
  auto inverseBindMatricesData =
      inverseBindMatricesAccessor
          ->typed_data<glTF::accessor::component_type::the_float>();
  for (decltype(inverseBindMatrices.size()) i = 0;
       i < inverseBindMatrices.size(); ++i) {
    _convertMaxMatrix3ToGlTFMat4(inverseBindMatrices[i],
                                 inverseBindMatricesData + 16 * i);
  }
  glTFSkin->inverse_bind_matrices(inverseBindMatricesAccessor);

  return glTFSkin;
}
} // namespace fant