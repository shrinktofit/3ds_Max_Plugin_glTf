
#include <array>
#include <fant/3ds_Max_classes/exporter_ui.h>
#pragma warning(push)
#pragma warning(disable : 4996)
#include <wx/app.h>
#include <wx/checkbox.h>
#include <wx/combobox.h>
#include <wx/dialog.h>
#include <wx/frame.h>
#include <wx/panel.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#pragma warning(pop)

namespace fant {
[[nodiscard]] inline wxString operator""_wx(const char8_t *str_, size_t len_) {
  return wxString::FromUTF8(reinterpret_cast<const char *>(str_), len_);
}

class export_dialog_frame : public wxFrame {
public:
  export_dialog_frame() : wxFrame(nullptr, wxID_ANY, "Exporter") {
    auto boxSizer = new wxBoxSizer(wxVERTICAL);

    auto gridSizer = new wxFlexGridSizer(2, {
                                                5, // column gap
                                                2, // row gap
                                            });

    wxArrayString formatChoices;
    formatChoices.assign(
        {u8"Mixed(.gltf + .bin)"_wx, u8"Binary(.glb)"_wx, u8"JSON(.gltf)"_wx});
    auto formatCombox =
        new wxComboBox(this, wxID_ANY, wxEmptyString, wxDefaultPosition,
                       wxDefaultSize, formatChoices, wxCB_SORT);
    formatCombox->SetSelection(0);
    formatCombox->SetEditable(false);

    gridSizer->Add(new wxStaticText(this, -1, u8"Format"_wx));
    gridSizer->Add(formatCombox);

    wxArrayString vertexIndexChoices;
    vertexIndexChoices.assign({
        u8"Auto"_wx,
        u8"Fixed 1 bytes"_wx,
        u8"Fixed 2 bytes"_wx,
        u8"Fixed 4 bytes"_wx,
    });
    auto vertexIndexComboBox =
        new wxComboBox(this, wxID_ANY, wxEmptyString, wxDefaultPosition,
                       wxDefaultSize, vertexIndexChoices, wxCB_SORT);
    vertexIndexComboBox->SetSelection(0);
    vertexIndexComboBox->SetEditable(false);
    gridSizer->Add(new wxStaticText(this, -1, u8"Vertex index"_wx));
    gridSizer->Add(vertexIndexComboBox);

    boxSizer->Add(gridSizer); // sizer->Add(gs, 1, wxEXPAND);

    auto checkBox =
        new wxCheckBox(this, wxID_ANY, u8"KHR Draco mesh compression"_wx);
    boxSizer->Add(checkBox);

    SetSizer(boxSizer);
  }

  /*void OnExit(wxCommandEvent &event) override {
    Close(true);
  }*/
};

class export_dialog : public wxApp {
public:
  bool OnInit() override {
    _frame = new export_dialog_frame();
    _frame->Show(true);
    return true;
  }

private:
  export_dialog_frame *_frame = nullptr;
};

std::optional<export_settings> open_export_dialog() {
  int argc = 0;
  char **argv = nullptr;
  export_dialog *app = new export_dialog();
  wxApp::SetInstance(app);
  auto code = wxEntry(argc, argv);
  return std::optional<export_settings>();
}
} // namespace fant