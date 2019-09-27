
#include <bitmap.h>
#include <fant/3ds_Max_classes/exporter_visitor.h>
#include <stdmat.h>

namespace fant {
std::vector<glTF::object_ptr<glTF::material>>
exporter_visitor::_tryExportMaterial(INode &max_node_) {
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
exporter_visitor::_tryConvertMaterial(Mtl &max_mtl_) {
  if (max_mtl_.ClassID() == Class_ID(MULTI_CLASS_ID, 0)) {
    return _convertMultiMaterial(reinterpret_cast<MultiMtl &>(max_mtl_));
  } else if (max_mtl_.ClassID() == Class_ID(DMTL_CLASS_ID, 0)) {
    return {_convertStdMaterial(reinterpret_cast<StdMat &>(max_mtl_))};
  }
  return {};
}

glTF::object_ptr<glTF::material>
exporter_visitor::_convertStdMaterial(StdMat &max_mtl_) {
  /*
  two = std->GetTwoSided();

     // Access the Diffuse map and see if it's a Bitmap texture
     Texmap *tmap = m->GetSubTexmap( ID_DI);
     if (tmap->ClassID() == Class_ID(BMTEX_CLASS_ID, 0))
     {
      // It is -- Access the UV tiling settings at time 0.
       BitmapTex *bmt = (BitmapTex*) tmap;
       StdUVGen *uv = bmt->GetUVGen();
      utile = uv->GetUScl(0);
      vtile = uv->GetVScl(0);
      buf.printf(_T("Two sided=%d, U Tile = %.1f, V Tile = %.1f"),
      two, utile, vtile);
      MessageBox(ip->GetMAXHWnd(), buf, _T("Info..."),
      MB_ICONINFORMATION);
     }
  */
  auto glTFMaterial = _document.factory().make<glTF::material>();
  glTFMaterial->name(_convertMaxName(max_mtl_.GetName()));

  glTFMaterial->double_sided(max_mtl_.GetTwoSided());

  glTF::material::pbr_metallic_roughness_info pbrInfo;

  auto diffuseColor = max_mtl_.GetDiffuse(0);
  pbrInfo.base_color_factor(diffuseColor.r, diffuseColor.g, diffuseColor.b);

  if (auto tmap = max_mtl_.GetSubTexmap(ID_DI)) {
    if (auto tinfo = _tryConvertTexture(*tmap)) {
      pbrInfo.base_color_texture(*tinfo);
    }
  }

  glTFMaterial->pbr_metallic_roughness(pbrInfo);

  return glTFMaterial;
}

std::vector<glTF::object_ptr<glTF::material>>
exporter_visitor::_convertMultiMaterial(MultiMtl &max_mtl_) {
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
      // TODO warn: submaterial of multimat shall not be multimat anymore.
    }
  }
  return submaterials;
}
} // namespace fant