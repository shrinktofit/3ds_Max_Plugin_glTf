
#include <apricot/exporter.h>
#include <apricot/exporter/export_settings.h>
#include <apricot/exporter/exporter.impl.h>
#include <apricot/exporter/ui.h>
#include <apricot/glTF.h>
#include <apricot/utilities/uri_utility.h>
#include <fstream>
#include <iostream>
#include <vector>

namespace apricot {
int exporter::do_export(const MCHAR *name,
                        ExpInterface *ei,
                        Interface *i,
                        BOOL suppressPrompts,
                        DWORD options) {
  namespace fs = std::filesystem;

  open_export_dialog();
  auto path = std::filesystem::path(name);

  export_settings settings;
  settings.format = export_settings::format_t::json;
  settings.index_type = export_settings::index_type_t::least_u8;
  settings.image_storage = export_settings::image_storage_t::standalone;

  glTF::document glTFDocument;
  exporter_impl visitor{*i, glTFDocument, settings, *ei->theScene};

  auto extStr = path.extension().string();
  auto extStrLower = extStr;
  std::transform(extStrLower.begin(), extStrLower.end(), extStrLower.begin(),
                 ::tolower);
  auto u8Name = path.u8string();
  bool isGlb = extStrLower == ".glb";

  // Assign url to each buffer and write them.
  if (settings.format == export_settings::format_t::json) {
    auto glTFFileDir = path.parent_path();
    auto nBuffers = glTFDocument.factory().get_size<glTF::buffer>();
    for (decltype(nBuffers) iBuffer = 0; iBuffer < nBuffers; ++iBuffer) {
      auto buffer = glTFDocument.factory().get<glTF::buffer>(iBuffer);
      if (auto uri = buffer->uri(); !uri) {
        fs::path bufferFileFullName =
            glTFFileDir /
            fs::path(path.stem().u8string() +
                     (nBuffers == 1 ? u8"" : to_u8string(iBuffer)) + u8".bin");

        std::basic_ofstream<std::byte> ofs(bufferFileFullName,
                                           std::ios::binary);
        std::vector<std::byte> data(buffer->size());
        buffer->read_all(data.data());
        ofs.write(data.data(), data.size());

        buffer->uri(
            uri_from_path(fs::relative(bufferFileFullName, glTFFileDir)));
      }
    }
  }

  auto glTFJsonStr = glTFDocument.serialize(isGlb).dump(2);
  if (isGlb) {
    std::vector<glTF::chunk> chunks;
    chunks.reserve(2);

    // The json chunk.
    chunks.emplace_back(glTF::make_json_chunk(glTFJsonStr));

    // If the first buffer has no uri, it's treated as a chunk.
    std::optional<std::vector<std::byte>> firstBufferData;
    if (glTFDocument.factory().get_size<glTF::buffer>() != 0) {
      auto firstBuffer = glTFDocument.factory().get<glTF::buffer>(0);
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

  return IMPEXP_SUCCESS;
}

BOOL exporter::supports_options(int ext, DWORD options) {
  return FALSE;
}
} // namespace apricot