
#define NOMINMAX
#include "resource.h"
#include <array>
#include <charconv>
#include <decomp.h>
#include <filesystem>
#include <fstream>
#include <gsl/span>
#include <list>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <string_view>
#include <tapu/3ds_Max_classes/exporter.h>
#include <tapu/3ds_Max_classes/exporter/export_settings.h>
#include <tapu/3ds_Max_classes/exporter/glTF_creator.h>
#include <tapu/support/win32/get_string_resource.h>
#include <tapu/support/win32/instance.h>
#include <tapu/support/win32/mchar_to_utf8.h>
#include <unordered_map>
#include <vector>

namespace tapu {
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

class main_visitor : public ITreeEnumProc {
public:
  main_visitor(glTf_creator &creator_)
      : _creator(creator_), _sceneHandle(_creator.add_scene()) {
    _creator.set_default_scene(_sceneHandle);
  }

  int callback(INode *max_node_) override {
    auto glTfNode = _creator.add_node(*max_node_);
    if (!max_node_->IsRootNode()) {
      auto parentMaxNode = max_node_->GetParentNode();
      auto rParentGlTfNode = _nodeMaps.find(parentMaxNode);
      if (rParentGlTfNode != _nodeMaps.end()) {
        _creator.set_parent(glTfNode, rParentGlTfNode->second);
      } else {
        _creator.set_root_node(_sceneHandle, glTfNode);
      }
    }
    auto object = max_node_->EvalWorldState(0).obj;
    if (object->CanConvertToType({TRIOBJ_CLASS_ID, 0})) {
      auto triObject = static_cast<TriObject *>(
          object->ConvertToType(0, {TRIOBJ_CLASS_ID, 0}));
      if (triObject) {
        auto glTfMesh = _creator.add_tri_object(*triObject);
        _creator.set_mesh(glTfNode, glTfMesh);
        if (triObject != object) {
          triObject->DeleteMe();
        }
      }
    }
    _nodeMaps.emplace(max_node_, glTfNode);
    _creator.add_node_animation(*max_node_, glTfNode);
    return TREE_CONTINUE;
  }

private:
  glTf_creator &_creator;
  std::unordered_map<INode *, glTf_creator::node_handle> _nodeMaps;
  glTf_creator::scene_handle _sceneHandle;
};

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

int glTf_exporter::DoExport(const MCHAR *name,
                            ExpInterface *ei,
                            Interface *i,
                            BOOL suppressPrompts,
                            DWORD options) {
  auto path = std::filesystem::path(name);

  export_settings settings;
  settings.index = export_settings::index_setting::at_least;
  settings.index_type = export_settings::index_type_setting::unsigned_8;

  glTf_creator creator{settings, *i};

  main_visitor visitor{creator};
  ei->theScene->EnumTree(&visitor);
  creator.commit();

  auto extStr = path.extension().string();
  auto extStrLower = extStr;
  std::transform(extStrLower.begin(), extStrLower.end(), extStrLower.begin(),
                 ::tolower);
  auto u8Name = path.u8string();
  if (extStrLower == ".glb") {
    creator.save(u8Name);
  } else {
    creator.save(u8Name, path.stem().u8string(), u8".BIN");
  }
  return 1;
}

BOOL glTf_exporter::SupportsOptions(int ext, DWORD options) {
  return 1;
}
} // namespace tapu