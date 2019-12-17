
#include "resource.h"
#include <apricot/exporter.h>
#include <apricot/exporter/export_settings.h>
#include <array>
#include <condition_variable>
#include <glTF_plugin/exporter.h>
#include <glTF_plugin/utilities/win32/get_string_resource.h>
#include <glTF_plugin/utilities/win32/instance.h>
#include <ifnpub.h>
#include <iostream>
#include <maxscript/foundation/WindowStream.h>
#include <maxscript/macros/define_instantiation_functions.h>
#include <maxscript/maxscript.h>
#include <mutex>
#include <optional>
#include <tchar.h>

#define GLTF_EXPORT_DIALOG_INTERFACE_ID Interface_ID(0x38646bc2, 0x618d686b)

class glTF_export_dialog_interface_t : public FPStaticInterface {
public:
  enum class function_id {
    id_get_dialog_hwnd,
    close,
  };

  virtual HWND get_dialog_hwnd() {
    return dialog_hwnd;
  }

  virtual void close() {
    closed = true;
  }

  bool closed = true;

  HWND dialog_hwnd = nullptr;

  DECLARE_DESCRIPTOR(glTF_export_dialog_interface_t);
  BEGIN_FUNCTION_MAP
  FN_0(function_id::id_get_dialog_hwnd, TYPE_HWND, get_dialog_hwnd)
  VFN_0(function_id::close, close)
  END_FUNCTION_MAP
};

static glTF_export_dialog_interface_t glTF_export_dialog_interface(
    GLTF_EXPORT_DIALOG_INTERFACE_ID, // id
    _T("glTFExportDialogInterface"), // name
    IDS_GLTF_EXPORTER_AUTHOR_NAME,   // description resource id
    NULL,                            // class description
    FP_CORE,                         // flags
    // close
    glTF_export_dialog_interface_t::function_id::close, // method id
    _T("close"),                                        // method name
    0,
    TYPE_VOID, // return type
    0,
    0, // parameter count
       // getDialogHwnd
    glTF_export_dialog_interface_t::function_id::id_get_dialog_hwnd, // method
                                                                     // id
    _T("getDialogHwnd"), // method name
    0,
    TYPE_HWND, // return type
    0,
    0,      // parameter count
    p_end); // indicates parameter specification ends

void open_export_dialog(HWND max_hwmd_) {
  auto uiScript = _T(R"xxx(
(
	rollout glTFExportDialog "glTF Exporter" (
		dropdownlist dropdownlistFormat "Format" items:#("Mixed(.gltf + .bin)", "Binary(.glb)", "JSON(.gltf)")
		dropdownlist dropdownlistIndexType "Index type" items:#("At least 8 bits", "At least 16 bits", "8 bits", "16 bits", "32 bits")
		dropdownlist dropdownlistImageStorage "Image storage" items:#("Standalone", "Embedded as binary", "Embedded as URI")

		button buttonExport "Export"
		button buttonCancel "Cancel"
		
		on buttonExport pressed do (
      glTFExportDialogInterface.close()
			destroydialog glTFExportDialog
		)
		
		on buttonCancel pressed do (
      glTFExportDialogInterface.close()
			destroydialog glTFExportDialog
		)
	)
	glTFExportDialogInstance = createDialog glTFExportDialog modal
)
)xxx");

  glTF_export_dialog_interface.closed = false;

  FPValue fpHwnd;
  ExecuteMAXScriptScript(uiScript,
                         0, // quietErrors
                         &fpHwnd);

  MSG message;
  HWND hwnd = nullptr;
  while (!glTF_export_dialog_interface.closed) {
    if (GetMessage(&message, hwnd, 0, 0)) {
      TranslateMessage(&message);
      DispatchMessage(&message);
    }
  }
}

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
  if (true || !suppressPrompts) {
    open_export_dialog(i->GetMAXHWnd());
  }
  apricot::export_settings settings;
  return _impl->exporter.do_export(name, ei, i, suppressPrompts, options,
                                   settings);
}

BOOL glTF_exporter::SupportsOptions(int ext, DWORD options) {
  return _impl->exporter.supports_options(ext, options);
}
} // namespace plugin