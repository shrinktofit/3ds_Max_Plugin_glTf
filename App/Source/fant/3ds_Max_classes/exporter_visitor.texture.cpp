#include <bitmap.h>
#include <fant/3ds_Max_classes/exporter_visitor.h>
#include <stdmat.h>

namespace fant {
std::optional<glTF::texture_info>
exporter_visitor::_tryConvertTexture(Texmap &tex_map_) {
  if (auto r = _textureMap.find(&tex_map_); r != _textureMap.end()) {
    return r->second;
  }
  if (tex_map_.ClassID() == Class_ID(BMTEX_CLASS_ID, 0)) {
    auto &bmt = reinterpret_cast<BitmapTex &>(tex_map_);
    auto glTFTexture = _document.factory().make<glTF::texture>();
    glTFTexture->name(_convertMaxName(bmt.GetName()));
    bmt.GetBitmap(0)->GetBitmapInfo();
    return {
        glTFTexture,
    };
  }
  return std::nullopt;
}
} // namespace fant