
#include "resource.h"
#include <apricot/exporter.h>
#include <array>
#include <glTF_plugin/exporter.h>
#include <glTF_plugin/utilities/win32/get_string_resource.h>
#include <glTF_plugin/utilities/win32/instance.h>

namespace plugin {
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

void *glTF_exporter::class_description::Create(BOOL) {
  return new glTF_exporter();
}

const MCHAR *glTF_exporter::class_description::ClassName() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_CLASS_NAME);
}

Class_ID glTF_exporter::class_description::ClassID() {
  static Class_ID id = Class_ID(0x4b4b76a0, 0x68434828);
  return id;
}

const MCHAR *glTF_exporter::class_description::Category() {
  return _M("");
}

const MCHAR *glTF_exporter::class_description::InternalName() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_INTERNAL_NAME);
}

HINSTANCE glTF_exporter::class_description::HInstance() {
  return win32::get_instance();
}

class glTF_exporter::_impl_t {
public:
  apricot::exporter exporter;
};

glTF_exporter::glTF_exporter() : _impl(std::make_unique<_impl_t>()) {
}

glTF_exporter::~glTF_exporter() {
}

int glTF_exporter::ExtCount() {
  return static_cast<int>(glTf_extension_info_list.size());
}

const MCHAR *glTF_exporter::Ext(int i_) {
  if (i_ < 0 || i_ >= glTf_extension_info_list.size()) {
    return _M("");
  } else {
    return glTf_extension_info_list[i_].rep;
  }
}

const MCHAR *glTF_exporter::LongDesc() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_LONG_DESCRIPTION);
}

const MCHAR *glTF_exporter::ShortDesc() {
  auto result = win32::get_string_resource(IDS_GLTF_EXPORTER_SHORT_DESCRIPTION);
  return result;
}

const MCHAR *glTF_exporter::AuthorName() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_AUTHOR_NAME);
}

const MCHAR *glTF_exporter::CopyrightMessage() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_COPYRIGHT_MESSAGE);
}

const MCHAR *glTF_exporter::OtherMessage1() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_OTHER_MESSAGE_1);
}

const MCHAR *glTF_exporter::OtherMessage2() {
  return win32::get_string_resource(IDS_GLTF_EXPORTER_OTHER_MESSAGE_2);
}

unsigned int glTF_exporter::Version() {
  return 100;
}

void glTF_exporter::ShowAbout(HWND hWnd) {
  MessageBox(hWnd, win32::get_string_resource(IDS_GLTF_EXPORTER_ABOUT),
             _M("About"), MB_OK);
}

int glTF_exporter::DoExport(const MCHAR *name,
                            ExpInterface *ei,
                            Interface *i,
                            BOOL suppressPrompts,
                            DWORD options) {
  return _impl->exporter.do_export(name, ei, i, suppressPrompts, options);
}

BOOL glTF_exporter::SupportsOptions(int ext, DWORD options) {
  return _impl->exporter.supports_options(ext, options);
}
} // namespace plugin