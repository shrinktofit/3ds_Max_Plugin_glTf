
#include <IGame/IGameModifier.h>
#include <fant/3ds_Max_classes/exporter_visitor.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_decompose.hpp>

namespace fant {
std::pair<glTF::object_ptr<glTF::skin>, exporter_visitor::_vertex_skin_data>
exporter_visitor::_exportSkin(IGameNode &igame_node_,
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

  decltype(_vertex_skin_data::sets) sets;

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

  bool exceedMaxAllowedJoints = false;
  auto addset = [&]() {
    auto iset = sets.size();
    auto j = std::make_unique<std::byte[]>(sizeof(JointStorage) * 4 * nVerts);
    std::fill_n(reinterpret_cast<JointStorage *>(j.get()), 4 * nVerts,
                JointStorage(-1));
    auto w = std::make_unique<std::byte[]>(sizeof(WeightStorage) * 4 * nVerts);
    sets.emplace_back(
        _vertex_skin_data::jw_channel{std::move(j), std::move(w)});
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

  auto nSkinnedVertices = igame_skin_.GetNumOfSkinnedVerts();

  auto readJW = [&](decltype(nSkinnedVertices) vertex_index_,
                    decltype(igame_skin_.GetNumberOfBones(
                        nSkinnedVertices)) affecting_bone_index_) {
    auto iset = affecting_bone_index_ / 4;
    auto iinflu = affecting_bone_index_ % 4;

    auto affectingBoneId =
        igame_skin_.GetBoneID(vertex_index_, affecting_bone_index_);
    auto rBoneArrayIndex = boneMap.find(affectingBoneId);
    if (rBoneArrayIndex == boneMap.end()) {
      // TODO: warn
      return;
    }

    auto boneArrayIndex = rBoneArrayIndex->second;
    assert(boneArrayIndex >= 0);
    reinterpret_cast<JointStorage *>(
        sets[iset].joints.get())[4 * vertex_index_ + iinflu] = boneArrayIndex;

    auto weight = igame_skin_.GetWeight(vertex_index_, affecting_bone_index_);
    if (weight <= 0) {
      return;
    }
    reinterpret_cast<WeightStorage *>(
        sets[iset].weights.get())[4 * vertex_index_ + iinflu] = weight;
  };

  for (decltype(nSkinnedVertices) iSkinnedVertex = 0;
       iSkinnedVertex < nSkinnedVertices; ++iSkinnedVertex) {
    auto vertexType = igame_skin_.GetVertexType(iSkinnedVertex);
    if (vertexType == IGameSkin::VertexType::IGAME_RIGID) {
      readJW(iSkinnedVertex, 0);
    } else {
      // vertexType == IGameSkin::VertexType::IGAME_RIGID_BLENDED
      const auto nEffectingBones = igame_skin_.GetNumberOfBones(iSkinnedVertex);
      ensureSets(nEffectingBones);
      for (std::remove_const_t<decltype(nEffectingBones)> iAffectingBone = 0;
           iAffectingBone != nEffectingBones; ++iAffectingBone) {
        readJW(iSkinnedVertex, iAffectingBone);
      }
    }
  }

  if (exceedMaxAllowedJoints) {
    // TODO warning
  }

  _vertex_skin_data vertexSkinData;
  vertexSkinData.joint_storage = glTF::accessor::component_type::unsigned_short;
  vertexSkinData.weight_storage = glTF::accessor::component_type::the_float;
  vertexSkinData.sets = std::move(sets);
#ifdef DEBUG_TPOSE
  vertexSkinData.bindposes = inverseBindMatrices;
#endif
  return {glTFSkin, std::move(vertexSkinData)};
}
} // namespace fant