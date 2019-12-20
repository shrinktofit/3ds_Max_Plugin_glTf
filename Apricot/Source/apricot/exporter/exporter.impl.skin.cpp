
#include <IGame/IGameModifier.h>
#include <apricot/exporter/exporter.impl.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_decompose.hpp>

namespace apricot {
std::pair<glTF::object_ptr<glTF::skin>, exporter_impl::skin_statistics>
exporter_impl::_exportSkin(IGameNode &igame_node_,
                           IGameSkin &igame_skin_,
                           glTF::object_ptr<glTF::node> glTF_mesh_node_) {
  // https://github.com/OGRECave/EasyOgreExporter/blob/master/source/ExSkeleton.cpp
  // https://github.com/homer6/c_reading/blob/4dc6b608203bb0a053c703d80b6af5e1141983ab/cat_mother/maxexport/SgUtil.cpp
  auto glTFSkin = _document.factory().make<glTF::skin>();

  auto meshNodeWorldTM = _calculateWorldMatrix(glTF_mesh_node_);

  // Bones that affecting vertices.
  auto nBones = igame_skin_.GetTotalBoneCount();

  // Bones that assigned to the skin modifier.
  auto nSkinBones = igame_skin_.GetTotalSkinBoneCount();

  GMatrix initSkinTM; // Transform mesh to world.
  igame_skin_.GetInitSkinTM(initSkinTM);

  std::vector<std::string> boneNames(nBones);
  for (std::remove_const_t<decltype(nBones)> iBone = 0; iBone < nBones;
       ++iBone) {
    auto boneNode = igame_skin_.GetIGameBone(iBone, true);
    auto rglTFBoneNode = _nodeMaps2.find(boneNode);
    if (rglTFBoneNode == _nodeMaps2.end()) {
      throw std::runtime_error("Bone node is not in the scene graph.");
    }
    std::u8string_view name = rglTFBoneNode->second->name();
    boneNames[iBone] = std::string(name.begin(), name.end());
  }
  std::unordered_map<decltype(igame_skin_.GetBoneID(0, 0)), glTF::integer>
      boneMap;
  std::vector<glm::mat4> inverseBindMatrices(nBones);
  for (std::remove_const_t<decltype(nBones)> iBone = 0; iBone < nBones;
       ++iBone) {
    auto boneNode = igame_skin_.GetIGameBone(iBone, true);
    auto rglTFBoneNode = _nodeMaps2.find(boneNode);
    if (rglTFBoneNode == _nodeMaps2.end()) {
      throw std::runtime_error("Bone node is not in the scene graph.");
    }
    auto glTFBoneNode = rglTFBoneNode->second;

    // auto boneNodeWorldTM = boneNode->GetWorldTM(0);
    // auto meshNodeWorldTM = igame_node_.GetWorldTM(0);
    // GMatrix initSkinTM; // Transform mesh to world.
    // igame_skin_.GetInitSkinTM(initSkinTM);
    // GMatrix
    //    initBoneTM; // Bone's world node TM. (equalavent to
    //    `boneNodeWorldTM`?)
    // bool successed = igame_skin_.GetInitBoneTM(boneNode, initBoneTM);
    // GMatrix inverseInitBoneTM = GMatrix(initBoneTM).Inverse();

    auto initBoneTM = _calculateWorldMatrix(glTFBoneNode);
    auto inverseInitBoneTM = glm::inverse(initBoneTM);

    auto inverseBindMatrix = inverseInitBoneTM * meshNodeWorldTM;

    glm::vec3 t;
    glm::vec3 s;
    glm::quat r;
    glm::vec3 skew;
    glm::vec4 per;
    auto decomposeresult =
        glm::decompose(inverseBindMatrix, s, r, t, skew, per);
    std::cout << decomposeresult;

    inverseBindMatrices[iBone] = inverseBindMatrix;

    glTFSkin->add_joint(glTFBoneNode);

    boneMap.emplace(boneNode->GetNodeID(), iBone);
  }

  using JointStorage = glTF::accessor::component_storage_t<
      glTF::accessor::component_type::unsigned_short>;

  using WeightStorage = glTF::accessor::component_storage_t<
      glTF::accessor::component_type::the_float>;

  auto nVerts = igame_skin_.GetNumOfSkinnedVerts();

  auto inverseBindMatricesAccessor = _makeSimpleAccessor(
      glTF::accessor::type_type::mat4,
      glTF::accessor::component_type::the_float, inverseBindMatrices.size());
  auto inverseBindMatricesData =
      inverseBindMatricesAccessor
          ->typed_data<glTF::accessor::component_type::the_float>();
  for (decltype(inverseBindMatrices.size()) i = 0;
       i < inverseBindMatrices.size(); ++i) {
    std::copy_n(glm::value_ptr(inverseBindMatrices[i]), 16,
                inverseBindMatricesData + 16 * i);
  }
  glTFSkin->inverse_bind_matrices(inverseBindMatricesAccessor);

  std::size_t maxPossibleInflunces = 0;
  // Count max influence of per vertex.
  auto nSkinnedVertices = igame_skin_.GetNumOfSkinnedVerts();
  for (decltype(nSkinnedVertices) iSkinnedVertex = 0;
       iSkinnedVertex < nSkinnedVertices; ++iSkinnedVertex) {
    auto vertexType = igame_skin_.GetVertexType(iSkinnedVertex);
    if (vertexType == IGameSkin::VertexType::IGAME_RIGID) {
      maxPossibleInflunces =
          std::max(maxPossibleInflunces, decltype(maxPossibleInflunces)(1));
    } else {
      // vertexType == IGameSkin::VertexType::IGAME_RIGID_BLENDED
      maxPossibleInflunces =
          std::max(maxPossibleInflunces,
                   decltype(maxPossibleInflunces)(
                       igame_skin_.GetNumberOfBones(iSkinnedVertex)));
    }
  }

  // If the limit is beyond, we have to reduce.
  auto nInfluences = 0;
  bool influencesMayOverrange = false;
  if (!_settings.max_joint_influence ||
      *_settings.max_joint_influence >= maxPossibleInflunces) {
    nInfluences = maxPossibleInflunces;
  } else {
    // TODO LOG
    nInfluences = *_settings.max_joint_influence;
    influencesMayOverrange = true;
  }

  // Allocate space for channels.
  constexpr auto influenceUnitSize = 4;
  auto influenceCapacity = (nInfluences / influenceUnitSize +
                            (nInfluences % influenceUnitSize ? 1 : 0)) *
                           influenceUnitSize;

  using VertexIndex = decltype(nSkinnedVertices);
  using WeightIn = float;
  using NumberOfAffectingBones = decltype(igame_skin_.GetNumberOfBones(0));
  using InflunceIndex = std::size_t;

  skin_statistics vertexSkinData(influenceCapacity, nVerts);

  auto readJW = [&](InflunceIndex influnce_index_,
                    decltype(nSkinnedVertices) vertex_index_,
                    NumberOfAffectingBones affecting_bone_index_,
                    WeightIn weight_) {
    auto affectingBoneId =
        igame_skin_.GetBoneID(vertex_index_, affecting_bone_index_);
    auto rBoneArrayIndex = boneMap.find(affectingBoneId);
    if (rBoneArrayIndex == boneMap.end()) {
      // TODO: warn
      return;
    }

    auto boneArrayIndex = rBoneArrayIndex->second;
    assert(boneArrayIndex >= 0);
    if (weight_ <= 0) {
      return;
    }

    vertexSkinData.set_influence(vertex_index_, influnce_index_, boneArrayIndex,
                                 weight_);
  };

  auto sortedInfluences =
      std::make_unique<std::pair<NumberOfAffectingBones, WeightIn>[]>(
          maxPossibleInflunces);
  for (decltype(nSkinnedVertices) iSkinnedVertex = 0;
       iSkinnedVertex < nSkinnedVertices; ++iSkinnedVertex) {
    auto vertexType = igame_skin_.GetVertexType(iSkinnedVertex);
    if (vertexType == IGameSkin::VertexType::IGAME_RIGID) {
      readJW(0, iSkinnedVertex, 0, igame_skin_.GetWeight(iSkinnedVertex, 0));
    } else {
      // vertexType == IGameSkin::VertexType::IGAME_RIGID_BLENDED
      const auto nEffectingBones = igame_skin_.GetNumberOfBones(iSkinnedVertex);
      if (!influencesMayOverrange || nEffectingBones <= influenceCapacity) {
        for (std::remove_const_t<decltype(nEffectingBones)> iAffectingBone = 0;
             iAffectingBone != nEffectingBones; ++iAffectingBone) {
          readJW(iAffectingBone, iSkinnedVertex, iAffectingBone,
                 igame_skin_.GetWeight(iSkinnedVertex, iAffectingBone));
        }
      } else {
        // Sort bones.
        for (std::remove_const_t<decltype(nEffectingBones)> iAffectingBone = 0;
             iAffectingBone != nEffectingBones; ++iAffectingBone) {
          sortedInfluences[iAffectingBone] = std::make_pair(
              iAffectingBone,
              igame_skin_.GetWeight(iSkinnedVertex, iAffectingBone));
        }
        std::sort(sortedInfluences.get(),
                  sortedInfluences.get() + nEffectingBones,
                  [&](auto b1_, auto b2_) { return b1_.second < b2_.second; });

        WeightIn weightSum = 0.0;
        for (std::remove_const_t<decltype(nEffectingBones)> iAffectingBone = 0;
             iAffectingBone != influenceCapacity; ++iAffectingBone) {
          weightSum += sortedInfluences[iAffectingBone].second;
        }

        // Only the bones with highest weights are used.
        for (std::remove_const_t<decltype(influenceCapacity)> iInflunce = 0;
             iInflunce != influenceCapacity; ++iInflunce) {
          readJW(iInflunce, iSkinnedVertex, sortedInfluences[iInflunce].first,
                 sortedInfluences[iInflunce].second / weightSum);
        }
      }
    }
  }

#ifdef DEBUG_TPOSE
  vertexSkinData.bindposes = inverseBindMatrices;
#endif
  return {glTFSkin, std::move(vertexSkinData)};
}
} // namespace apricot