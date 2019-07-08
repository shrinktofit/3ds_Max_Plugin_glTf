
#include "resource.h"
#include <Windows.h>
#include <iparamb2.h>
#include <tapu/3ds_Max_Plugin_glTf.h>
#include <tapu/3ds_Max_classes/exporter.h>
#include <tapu/support/win32/get_string_resource.h>
#include <tapu/support/win32/instance.h>
#include <tuple>

using class_descriptions_t = std::tuple<tapu::glTf_exporter::class_description>;

int controlsInit = FALSE;

// This function is called by Windows when the DLL is loaded.  This
// function may also be called many times during time critical operations
// like rendering.  Therefore developers need to be careful what they
// do inside this function.  In the code below, note how after the DLL is
// loaded the first time only a few statements are executed.

BOOL WINAPI DllMain(HINSTANCE hinstDLL,
                    ULONG fdwReason,
                    LPVOID /*lpvReserved*/) {
  if (fdwReason == DLL_PROCESS_ATTACH) {
    // Hang on to this DLL's instance handle.
    tapu::win32::set_instance(hinstDLL);
    DebugPrint(_T("glTf plugin loaded."));
    DisableThreadLibraryCalls(tapu::win32::get_instance());
  }
  return (TRUE);
}

// This function returns a string that describes the DLL and where the user
// could purchase the DLL if they don't have it.
__declspec(dllexport) const TCHAR *LibDescription() {
  return tapu::win32::get_string_resource(IDS_PLUGIN_DESCRIPTION);
}

// This function returns the number of plug-in classes this DLL
// TODO: Must change this number when adding a new class
__declspec(dllexport) int LibNumberClasses() {
  return std::tuple_size_v<class_descriptions_t>;
}

// This function returns the number of plug-in classes this DLL
__declspec(dllexport) ClassDesc *LibClassDesc(int i) {
  static class_descriptions_t descriptions;
  switch (i) {
  case 0:
    return &std::get<0>(descriptions);
  default:
    return 0;
  }
}

// This function returns a pre-defined constant indicating the version of
// the system under which it was compiled.  It is used to allow the system
// to catch obsolete DLLs.
__declspec(dllexport) ULONG LibVersion() {
  return VERSION_3DSMAX;
}

// This function is called once, right after your plugin has been loaded by 3ds
// Max. Perform one-time plugin initialization in this method. Return TRUE if
// you deem your plugin successfully loaded, or FALSE otherwise. If the function
// returns FALSE, the system will NOT load the plugin, it will then call
// FreeLibrary on your DLL, and send you a message.
__declspec(dllexport) int LibInitialize(void) {
  return 1; // TODO: Perform initialization here.
}

// This function is called once, just before the plugin is unloaded.
// Perform one-time plugin un-initialization in this method."
// The system doesn't pay attention to a return value.
__declspec(dllexport) int LibShutdown(void) {
  return 1; // TODO: Perform un-initialization here.
}
