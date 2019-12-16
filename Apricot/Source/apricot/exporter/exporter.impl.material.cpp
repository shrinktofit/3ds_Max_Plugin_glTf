
#include <bitmap.h>
#include <apricot/exporter/exporter.impl.h>
#include <stdmat.h>

namespace apricot {
std::vector<glTF::object_ptr<glTF::material>>
exporter_impl::_tryExportMaterial(INode &max_node_) {
  auto maxMaterial = max_node_.GetMtl();
  if (!maxMaterial) {
    return {};
  }
  auto converted = _materialMap.find(maxMaterial);
  if (converted != _materialMap.end()) {
    return converted->second;
  }
  auto glTFMaterial = _tryConvertMaterial(*maxMaterial);
  _materialMap.emplace(maxMaterial, glTFMaterial);
  return glTFMaterial;
}

std::vector<glTF::object_ptr<glTF::material>>
exporter_impl::_tryConvertMaterial(Mtl &max_mtl_) {
  if (max_mtl_.ClassID() == Class_ID(MULTI_CLASS_ID, 0)) {
    return _convertMultiMaterial(reinterpret_cast<MultiMtl &>(max_mtl_));
  } else if (max_mtl_.ClassID() == Class_ID(DMTL_CLASS_ID, 0)) {
    return {_convertStdMaterial(reinterpret_cast<StdMat &>(max_mtl_))};
  }
  return {};
}

glTF::object_ptr<glTF::material>
exporter_impl::_convertStdMaterial(StdMat &max_mtl_) {
  auto glTFMaterial = _document.factory().make<glTF::material>();
  glTFMaterial->name(_convertMaxName(max_mtl_.GetName()));

  glTFMaterial->double_sided(max_mtl_.GetTwoSided());

  glTF::material::pbr_metallic_roughness_info pbrInfo;

  auto diffuseColor = max_mtl_.GetDiffuse(0);
  auto opacity = max_mtl_.GetOpacity(0);
  pbrInfo.base_color_factor(diffuseColor.r, diffuseColor.g, diffuseColor.b,
                            opacity);
  if (opacity != 1) {
    glTFMaterial->alpha_mode(glTF::material::alpha_mode_type::blend);
  }

  if (auto tmap = max_mtl_.GetSubTexmap(ID_DI)) {
    if (auto tinfo = _tryConvertTexture(*tmap)) {
      pbrInfo.base_color_texture(*tinfo);
    }
  }

  glTFMaterial->pbr_metallic_roughness(pbrInfo);

  return glTFMaterial;
}

std::vector<glTF::object_ptr<glTF::material>>
exporter_impl::_convertMultiMaterial(MultiMtl &max_mtl_) {
  auto nSubMtls = max_mtl_.NumSubMtls();
  std::vector<glTF::object_ptr<glTF::material>> submaterials(nSubMtls);
  for (decltype(max_mtl_.NumSubMtls()) iSubMtl = 0; iSubMtl < nSubMtls;
       ++iSubMtl) {
    auto subMaxMtl = max_mtl_.GetSubMtl(iSubMtl);
    assert(subMaxMtl);
    auto converted = _tryConvertMaterial(*subMaxMtl);
    if (converted.size() == 1) {
      submaterials[iSubMtl] = converted.front();
    } else if (!converted.empty()) {
      throw std::runtime_error("Submaterial should not be multi-material.");
    }
  }
  return submaterials;
}

std::vector<glTF::object_ptr<glTF::material>>
exporter_impl::_exportMaterial(IGameMaterial &igame_materail_) {
  if (igame_materail_.IsMultiType()) {
    auto nSubMaterials = igame_materail_.GetSubMaterialCount();
    std::vector<glTF::object_ptr<glTF::material>> subGlTFMaterials(
        nSubMaterials);
    for (decltype(nSubMaterials) iSubMaterial = 0; iSubMaterial < nSubMaterials;
         ++iSubMaterial) {
      auto subMaterial = igame_materail_.GetSubMaterial(iSubMaterial);
      auto subGlTFMaterial = _exportMaterial(*subMaterial);
      if (subGlTFMaterial.size() != 1) {
        throw std::runtime_error("Submaterial should not be multi-material.");
      }
      subGlTFMaterials[iSubMaterial] = subGlTFMaterial.front();
    }
    return subGlTFMaterials;
  } else {
    return _tryConvertMaterial(*igame_materail_.GetMaxMaterial());
  }
}
} // namespace fant