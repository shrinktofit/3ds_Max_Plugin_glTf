#include <bitmap.h>
#include <cppcodec/base64_rfc4648.hpp>
#include <fant/3ds_Max_classes/exporter_visitor.h>
#include <fant/support/uri_utility.h>
#include <filesystem>
#include <fstream>
#include <istdplug.h>
#include <regex>
#include <stdmat.h>

void processSaveBitmapRes(BMMRES res) {
  switch (res) {
  case BMMRES_ERRORTAKENCARE:
    throw std::runtime_error(
        "3ds Max could not find a device to handle the image.");
  case BMMRES_INTERNALERROR:
    throw std::runtime_error(
        "The IO module handling the image could not be opened for writing.");
  }
}

std::vector<std::byte>
read_binary_file(const std::filesystem::path &file_path_) {
  std::ifstream ifs(file_path_, std::ios::binary | std::ios::ate);
  ifs.exceptions(std::ios::failbit);
  auto end = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  auto size = static_cast<std::size_t>(end - ifs.tellg());
  assert(size >= 0);
  std::vector<std::byte> result(size);
  if (!ifs.read(reinterpret_cast<char *>(result.data()), result.size())) {
    throw std::runtime_error("Failed to read " + file_path_.string());
  }
  return result;
}

namespace fant {
std::optional<glTF::image::allowed_mime_type>
deduce_mime_type_from_extension(const std::filesystem::path &extension_) {
  std::optional<glTF::image::allowed_mime_type> mimeType;
  if (auto ext = extension_.string(); std::regex_match(
          ext, std::regex(R"(\.png)", std::regex::flag_type::icase))) {
    mimeType = glTF::image::allowed_mime_type::png;
  } else if (std::regex_match(
                 ext, std::regex(R"(\.jpe?g)", std::regex::flag_type::icase))) {
    mimeType = glTF::image::allowed_mime_type::jpeg;
  }
  return mimeType;
}

std::optional<glTF::texture_info>
exporter_visitor::_tryConvertTexture(Texmap &tex_map_) {
  if (auto r = _textureMap.find(&tex_map_); r != _textureMap.end()) {
    return r->second;
  }

  auto glTFTexture = _document.factory().make<glTF::texture>();
  glTFTexture->name(_convertMaxName(tex_map_.GetName()));

  if (tex_map_.ClassID() == Class_ID(BMTEX_CLASS_ID, 0)) {
    auto &bmt = reinterpret_cast<BitmapTex &>(tex_map_);
    auto &bitmap = *bmt.GetBitmap(0);
    auto bitmapInfo = bitmap.GetBitmapInfo();

    auto glTFImage = _document.factory().make<glTF::image>();

    if (false) {
      namespace fs = std::filesystem;
      auto tmpDir = fs::path("X:\\Temp");
      auto tmpImgFile = tmpDir / "fuck.tga";

      BitmapInfo exportBitmapInfo;
      // exportBitmapInfo.Copy(&bitmapInfo);
      exportBitmapInfo.SetName(L"file.tga");
      exportBitmapInfo.SetWidth(512);
      exportBitmapInfo.SetHeight(512);
      /*exportBitmapInfo.SetFirstFrame(0);
      exportBitmapInfo.SetEndFrame(0);*/
      //  exportBitmapInfo.SetName(tmpDir.wstring().c_str());
      auto exportBitmap = TheManager->Create(&exportBitmapInfo);
      exportBitmap->CopyImage(&bitmap, COPY_IMAGE_RESIZE_HI_QUALITY, 0);
      auto res = exportBitmap->OpenOutput(&exportBitmapInfo);
      processSaveBitmapRes(res);
      res = exportBitmap->Write(&exportBitmapInfo);
      processSaveBitmapRes(res);
      res = exportBitmap->Close(&exportBitmapInfo);
      processSaveBitmapRes(res);
      exportBitmap->DeleteThis();
      auto imageFileBuffer = read_binary_file(tmpImgFile);
      auto bufferView = _document.factory().make<glTF::buffer_view>(
          _mainBuffer, imageFileBuffer.size(), 0);
      std::copy_n(imageFileBuffer.data(), imageFileBuffer.size(),
                  bufferView->data());
      glTFImage->source(bufferView, glTF::image::allowed_mime_type::png);
    } else {
      if (auto fullPathMax = bitmapInfo.Name()) {
        auto fullPath = _convertMaxPath(fullPathMax);
        if (auto mimeType =
                deduce_mime_type_from_extension(fullPath.extension());
            !mimeType) {
          // TO DO error
        } else {
          auto imageFileBuffer = read_binary_file(fullPath);
          auto bufferView = _document.factory().make<glTF::buffer_view>(
              _mainBuffer, imageFileBuffer.size(), 0);
          std::copy_n(imageFileBuffer.data(), imageFileBuffer.size(),
                      bufferView->data());
          glTFImage->source(bufferView, *mimeType);

          /*auto uri = uri_from_path(fullPath);
          glTFImage->source(uri);*/
        }
      }
    }

    glTFTexture->source(glTFImage);
  }

  return {
      glTFTexture,
  };
}
} // namespace fant