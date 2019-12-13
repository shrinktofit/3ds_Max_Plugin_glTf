
#include "resource.h"
#include <array>
#include <charconv>
#include <fant/3ds_Max_classes/export_settings.h>
#include <fant/3ds_Max_classes/exporter.h>
#include <fant/3ds_Max_classes/exporter_ui.h>
#include <fant/3ds_Max_classes/exporter_visitor.h>

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

void do_export(const MCHAR *name,
               ExpInterface *ei,
               Interface *i,
               BOOL suppressPrompts,
               DWORD options) {
  open_export_dialog();
  auto path = std::filesystem::path(name);

  export_settings settings;
  settings.index = export_settings::index_setting::at_least;
  settings.index_type = export_settings::index_type_setting::unsigned_8;

  glTF::document glTFDocument;
  exporter_visitor visitor{*i, glTFDocument, settings, *ei->theScene};

  auto extStr = path.extension().string();
  auto extStrLower = extStr;
  std::transform(extStrLower.begin(), extStrLower.end(), extStrLower.begin(),
                 ::tolower);
  auto u8Name = path.u8string();
  bool isGlb = extStrLower == ".glb";
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

  // Write each buffer specified uri.
  auto nBuffers = glTFDocument.factory().get_size<glTF::buffer>();
  for (decltype(nBuffers) iBuffer = 0; iBuffer < nBuffers; ++iBuffer) {
    auto buffer = glTFDocument.factory().get<glTF::buffer>(iBuffer);
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
}

INT_PTR CALLBACK exporter_dialog_proc(HWND hWnd,
                                      UINT message,
                                      WPARAM wParam,
                                      LPARAM lParam) {
  switch (message) {
  case WM_INITDIALOG: {
    auto exporter = (glTf_exporter *)(lParam);
    CenterWindow(hWnd, GetParent(hWnd));
    auto hFormatComboBox = GetDlgItem(hWnd, IDC_COMBO1);
    ComboBox_AddString(hFormatComboBox, L"Mixed(.gltf + .bin");
    ComboBox_AddString(hFormatComboBox, L"Binary(.glb)");
    ComboBox_AddString(hFormatComboBox, L"JSON(.gltf)");
    ComboBox_SetCurSel(hFormatComboBox, 0);
    break;
  }
  case WM_COMMAND: {
    switch (LOWORD(wParam)) {
    case IDOK: {
      EndDialog(hWnd, 1);
      break;
    }
    case IDCANCEL: {
      EndDialog(hWnd, 0);
      break;
    }
    }
    break;
  }
  default:
    return FALSE;
  }
  return TRUE;
}

std::u8string get_config_filename(Interface &max_interface_) {
  // APP_PLUGCFG_DIR
  auto plugcfdDirMSTR = max_interface_.GetDir(APP_PLUGCFG_LN_DIR);
  auto result = win32::mchar_to_utf8(
      std::basic_string_view<MCHAR>(plugcfdDirMSTR, _tcslen(plugcfdDirMSTR)));
  return result;
}

int glTf_exporter::DoExport(const MCHAR *name,
                            ExpInterface *ei,
                            Interface *i,
                            BOOL suppressPrompts,
                            DWORD options) {
  // bool showPrompts = !suppressPrompts;
  // if (showPrompts) {
  //  auto success = DialogBoxParam(
  //      win32::get_instance(), MAKEINTRESOURCE(IDD_GLTF_EXPORTER_DIALOG),
  //      i->GetMAXHWnd(), exporter_dialog_proc, (LPARAM)this);

  //  /*if (!success) {
  //    return 1;
  //  }*/
  //}
  (void)get_config_filename(*i);
  do_export(name, ei, i, suppressPrompts, options);

  return 1;
}

BOOL glTf_exporter::SupportsOptions(int ext, DWORD options) {
  return 1;
}
} // namespace fant
