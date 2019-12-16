
#include <apricot/exporter/ui.h>
#include <array>
#include <maxscript/maxscript.h>

namespace apricot {
std::optional<export_settings> open_export_dialog() {
  auto uiScript = _T(R"xxx(
(
	rollout glTFExporter "glTF Exporter" (
		dropdownlist dropdownlistFormat "Format" items:#("Mixed(.gltf + .bin)", "Binary(.glb)", "JSON(.gltf)")
		dropdownlist dropdownlistIndexType "Index type" items:#("At least 8 bits", "At least 16 bits", "8 bits", "16 bits", "32 bits")
		dropdownlist dropdownlistImageStorage "Image storage" items:#("Standalone", "Embedded as binary", "Embedded as URI")

		button buttonExport "Export"
		button buttonCancel "Cancel"
		
		
		on buttonExport pressed do (
			destroydialog glTFExporter
		)
		
		on buttonCancel pressed do (
			destroydialog glTFExporter
		)
	)
	createDialog glTFExporter
)
)xxx");
  ExecuteMAXScriptScript(uiScript);
  // int argc = 0;
  // char **argv = nullptr;
  // export_dialog *app = new export_dialog();
  // wxApp::SetInstance(app);
  // auto code = wxEntry(argc, argv);
  return std::optional<export_settings>();
}
} // namespace apricot