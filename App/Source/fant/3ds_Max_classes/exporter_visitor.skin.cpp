
#include <IGame/IGameModifier.h>
#include <fant/3ds_Max_classes/exporter_visitor.h>

namespace fant {
glTF::object_ptr<glTF::skin>
exporter_visitor::_exportSkin(IGameNode &igame_node_,
                              IGameSkin &igame_skin_,
                              _immediate_mesh &imm_mesh_) {
  return nullptr;
#if false
  auto glTFSkin = _document.factory().make<glTF::skin>();

  auto nBones = igame_skin_.GetTotalBoneCount();
  std::unordered_map<decltype(igame_skin_.GetBoneID(0, 0)), glTF::integer>
      boneMap;
  std::vector<GMatrix> inverseBindMatrices(nBones);
  for (std::remove_const_t<decltype(nBones)> iBone = 0; iBone < nBones;
       ++iBone) {
    auto boneNode = igame_skin_.GetIGameBone(iBone);
    auto rglTFBoneNode = _nodeMaps2.find(boneNode);
    if (rglTFBoneNode == _nodeMaps2.end()) {
      throw std::runtime_error("Bone node is not in the scene graph.");
    }

    auto boneNodeWordTM = boneNode->GetWorldTM(0);
    GMatrix initBoneTM;
    igame_skin_.GetInitBoneTM(boneNode, initBoneTM);
    // auto inverseBindMatrix = igame_node_.GetWorldTM(0) * initBoneTM;

    auto inverseBindMatrix =
        boneNode->GetWorldTM(0).Inverse() * igame_node_.GetWorldTM(0);

    inverseBindMatrices[iBone] = inverseBindMatrix;

    auto glTFBoneNode = rglTFBoneNode->second;
    glTFSkin->add_joint(glTFBoneNode);

    boneMap.emplace(boneNode->GetNodeID(), iBone);
  }

  auto inverseBindMatricesAccessor = _makeSimpleAccessor(
      glTF::accessor::type_type::mat4,
      glTF::accessor::component_type::the_float, inverseBindMatrices.size());
  auto inverseBindMatricesData =
      inverseBindMatricesAccessor
          ->typed_data<glTF::accessor::component_type::the_float>();
  for (decltype(inverseBindMatrices.size()) i = 0;
       i < inverseBindMatrices.size(); ++i) {
    _convert(inverseBindMatrices[i], inverseBindMatricesData + 16 * i);
  }
  glTFSkin->inverse_bind_matrices(inverseBindMatricesAccessor);

  bool exceedMaxAllowedJoints = false;
  std::vector<std::pair<std::byte *, std::byte *>> sets;
  auto addset = [&]() {
    auto iset = sets.size();
    auto jointsChannelData = imm_mesh_.vertices.add_channel(
        glTF::standard_semantics::joints(iset), glTF::accessor::type_type::vec4,
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

  auto nVerts = imm_mesh_.vertices.vertex_count();
  for (std::remove_const_t<decltype(nVerts)> iVertex = 0; iVertex < nVerts;
       ++iVertex) {
    auto iFaceVertex = imm_mesh_.vertex_resort[iVertex];
    const auto nEffectingBones = igame_skin_.GetNumberOfBones(iFaceVertex);
    ensureSets(nEffectingBones);
    for (std::remove_const_t<decltype(nEffectingBones)> iAffectingBone = 0;
         iAffectingBone != nEffectingBones; ++iAffectingBone) {
      auto iset = iAffectingBone / 4;
      auto iinflu = iAffectingBone % 4;

      using JointStorage = glTF::accessor::component_storage_t<
          glTF::accessor::component_type::unsigned_short>;

      using WeightStorage = glTF::accessor::component_storage_t<
          glTF::accessor::component_type::the_float>;

      auto affectingBoneId = igame_skin_.GetBoneID(iFaceVertex, iAffectingBone);
      auto rBoneArrayIndex = boneMap.find(affectingBoneId);
      if (rBoneArrayIndex == boneMap.end()) {
        // TODO: warn
        continue;
      }

      auto boneArrayIndex = rBoneArrayIndex->second;
      reinterpret_cast<JointStorage *>(sets[iset].first)[4 * iVertex + iinflu] =
          boneArrayIndex;

      auto weight = igame_skin_.GetWeight(iVertex, iAffectingBone);
      reinterpret_cast<WeightStorage *>(
          sets[iset].second)[4 * iVertex + iinflu] = weight;
    }
  }
  if (exceedMaxAllowedJoints) {
    // TODO warning
  }

  return glTFSkin;
#endif
}
} // namespace fant