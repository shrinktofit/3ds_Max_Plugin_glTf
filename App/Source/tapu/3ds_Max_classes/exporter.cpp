
#define NOMINMAX
#include "resource.h"
#include <array>
#include <charconv>
#include <filesystem>
#include <fstream>
#include <fx/gltf.h>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string_view>
#include <tapu/3ds_Max_classes/exporter.h>
#include <tapu/support/win32/get_string_resource.h>
#include <tapu/support/win32/instance.h>
#include <tapu/support/win32/mchar_to_utf8.h>
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

class vertex_list {
public:
  using size_type = int;

  using map_channel_size_type = int;

  vertex_list(size_type size_, map_channel_size_type map_channel_count_)
      : _nVertices(size_), _nMaps(map_channel_count_) {
    _vertexChannel = std::make_unique<Point3[]>(_nMaps);
    _normalChannel = std::make_unique<Point3[]>(_nMaps);
    _mapChannels =
        std::make_unique<std::unique_ptr<Point3[]>[]>(map_channel_count_);
    for (map_channel_size_type iMapChannel = 0;
         iMapChannel < map_channel_count_; ++iMapChannel) {
      _mapChannels[iMapChannel] = std::make_unique<Point3[]>(_nMaps);
    }
  }

  size_type size() const {
    return _nVertices;
  }

  size_type map_channel_size() const {
    return _nMaps;
  }

  const Point3 &get_vertex(size_type index_) const {
    return _vertexChannel[index_];
  }

  void set_vertex(size_type index_, const Point3 &value_) {
    _vertexChannel[index_] = value_;
  }

  const Point3 &get_normal(size_type index_) const {
    return _normalChannel[index_];
  }

  void set_normal(size_type index_, const Point3 &value_) {
    _normalChannel[index_] = value_;
  }

  const Point3 &get_uvw(map_channel_size_type map_index_,
                        size_type index_) const {
    return _mapChannels[map_index_][index_];
  }

  void set_uvw(map_channel_size_type map_index_,
               size_type index_,
               const Point3 &value_) {
    _mapChannels[map_index_][index_] = value_;
  }

  std::pair<vertex_list, std::unique_ptr<size_type[]>> to_indexed() const {
    vertex_list resultVertexList{_nVertices, _nMaps};
    size_type nResultVertices = 0;
    auto indices = std::make_unique<size_type[]>(_nVertices);
    for (size_type iVertex = 0; iVertex < _nVertices; ++iVertex) {
      bool found = false;
      for (size_type jResultVertex = 0; jResultVertex < nResultVertices;
           ++jResultVertex) {
        if (_compareVertex(jResultVertex, *this, resultVertexList)) {
          indices[iVertex] = jResultVertex;
          found = true;
          break;
        }
      }
      if (!found) {
        _copyVertex(nResultVertices, resultVertexList, *this);
        indices[iVertex] = nResultVertices;
        ++nResultVertices;
      }
    }
    return {
        resultVertexList._copyFirstNVertices(nResultVertices),
        std::move(indices),
    };
  }

private:
  size_type _nVertices;
  map_channel_size_type _nMaps;
  std::unique_ptr<Point3[]> _vertexChannel;
  std::unique_ptr<Point3[]> _normalChannel;
  std::unique_ptr<std::unique_ptr<Point3[]>[]> _mapChannels;

  vertex_list _copyFirstNVertices(size_type n_) {
    vertex_list result{n_, _nMaps};
    std::copy_n(_vertexChannel.get(), n_, result._vertexChannel.get());
    std::copy_n(_normalChannel.get(), n_, result._normalChannel.get());
    for (map_channel_size_type iMap = 0; iMap < _nMaps; ++iMap) {
      std::copy_n(_mapChannels[iMap].get(), n_,
                  result._mapChannels[iMap].get());
    }
  }

  static bool _compareVertex(size_type vertex_index_,
                             const vertex_list &lhs_,
                             const vertex_list &rhs_) {
    assert(vertex_index_ < lhs_.size());
    assert(vertex_index_ < rhs_.size());
    if (lhs_.get_vertex(vertex_index_) != rhs_.get_vertex(vertex_index_)) {
      return false;
    }
    if (lhs_.get_normal(vertex_index_) != rhs_.get_normal(vertex_index_)) {
      return false;
    }
    for (map_channel_size_type iMap = 0; iMap < lhs_.map_channel_size();
         ++iMap) {
      if (lhs_.get_uvw(iMap, vertex_index_) !=
          rhs_.get_uvw(iMap, vertex_index_)) {
        return false;
      }
    }
    return true;
  }

  static void _copyVertex(size_type vertex_index_,
                          vertex_list &lhs_,
                          const vertex_list &rhs_) {
    assert(vertex_index_ < lhs_.size());
    assert(vertex_index_ < rhs_.size());
    lhs_.set_vertex(vertex_index_, rhs_.get_vertex(vertex_index_));
    lhs_.set_normal(vertex_index_, rhs_.get_normal(vertex_index_));
    for (map_channel_size_type iMap = 0; iMap < lhs_.map_channel_size();
         ++iMap) {
      lhs_.set_uvw(iMap, vertex_index_, rhs_.get_uvw(iMap, vertex_index_));
    }
  }
};

class glTf_creator {
public:
  glTf_creator() {
    fx::gltf::Buffer buffer;
    buffer.data.resize(1);
    assert(buffer.data.size() <=
           std::numeric_limits<decltype(buffer.byteLength)>::max());
    buffer.byteLength =
        static_cast<decltype(buffer.byteLength)>(buffer.data.size());
    _glTfDocument.buffers.push_back(buffer);
  }

  void add_tri_object(TriObject &tri_obj_) {
    auto &mesh = tri_obj_.mesh;

    const auto nFaces = mesh.getNumFaces();
    const auto nMaps = mesh.getNumMaps();
    const auto nVData = mesh.getNumVData();

    vertex_list vertices(nFaces * 3, nMaps);

    for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
         ++iFace) {
      auto &face = mesh.faces[iFace];
      for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
        auto iVertex = face.getVert(iFaceVert);
        auto indexIntoFlatMesh = iFace * 3 + iFaceVert;

        auto &vertex = mesh.getVert(iVertex);
        auto &normal = mesh.getNormal(iVertex);
        vertices.set_vertex(indexIntoFlatMesh, vertex);
        vertices.set_normal(indexIntoFlatMesh, normal);
      }
    }

    for (std::remove_const_t<decltype(nMaps)> iMap = 0; iMap < nMaps; ++iMap) {
      const auto mapFaces = mesh.mapFaces(iMap);
      const auto mapVerts = mesh.mapVerts(iMap);
      for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
           ++iFace) {
        auto &mapFace = mapFaces[iMap];
        for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
          auto iVertex = mapFace.getTVert(iFaceVert);
          auto indexIntoFlatMesh = iFace * 3 + iFaceVert;

          const auto &uvw = mapVerts[iVertex];
          vertices.set_uvw(iMap, indexIntoFlatMesh, uvw);
        }
      }
    }

    for (std::remove_const_t<decltype(nVData)> iVData = 0; iVData < nVData;
         ++iVData) {
      auto vData = mesh.vData[iVData];
    }

    auto [indexedVertices, indices] = vertices.to_indexed();
  }

  /// <summary>
  /// Save as .glb format.
  /// </summary>
  /// <param name="path_"></param>
  /// <param name="binary_dir_name_"></param>
  /// <param name="binary_ext_"></param>
  void save(std::string_view path_,
            std::string_view binary_dir_name_,
            std::string_view binary_ext_) {
    for (decltype(_glTfDocument.buffers)::size_type iBuffer = 0;
         iBuffer != _glTfDocument.buffers.size(); ++iBuffer) {
      auto &buffer = _glTfDocument.buffers[iBuffer];
      std::stringstream urlss;
      urlss << u8"./" << binary_dir_name_;
      if (_glTfDocument.buffers.size() != 1) {
        std::array<char, sizeof(iBuffer) * CHAR_BIT> indexChars;
        auto result = std::to_chars(
            indexChars.data(), indexChars.data() + indexChars.size(), iBuffer);
        assert(result.ec == std::errc());
        // TODO: indexString is not guarenteen as UTF8.
        auto indexString =
            std::string_view(indexChars.data(), result.ptr - indexChars.data());
        urlss << u8"-" << indexString;
      }
      urlss << binary_ext_;
      buffer.uri = std::filesystem::u8path(urlss.str()).u8string();
    }
    _doSave(path_, false);
  }

  /// <summary>
  /// Save as .gltf file.
  /// </summary>
  /// <param name="path_"></param>
  void save(std::string_view path_) {
    for (auto &buffer : _glTfDocument.buffers) {
      buffer.uri.clear();
    }
    _doSave(path_, true);
  }

private:
  fx::gltf::Document _glTfDocument;

  void _doSave(std::string_view path_, bool binary_) try {
    auto path = std::filesystem::u8path(path_).string();
    fx::gltf::Save(_glTfDocument, path, binary_);
  } catch (const std::exception &exception_) {
    DebugPrint(L"glTf-exporter: exception occured when save:");
    _printException(exception_);
  } catch (...) {
    DebugPrint(L"glTf-exporter: Unknown exception.");
  }

  void _printException(const std::exception &e, int level = 0) {
    std::cerr << std::string(level, ' ') << "exception: " << e.what() << '\n';
    try {
      std::rethrow_if_nested(e);
    } catch (const std::exception &e) {
      _printException(e, level + 1);
    } catch (...) {
    }
  }
};

class main_visitor : public ITreeEnumProc {
public:
  main_visitor(glTf_creator &creator_) : _creator(creator_) {
  }

  int callback(INode *max_node_) override {
    auto object = max_node_->EvalWorldState(0).obj;
    if (object->CanConvertToType({TRIOBJ_CLASS_ID, 0})) {
      auto triObject = static_cast<TriObject *>(
          object->ConvertToType(0, {TRIOBJ_CLASS_ID, 0}));
      if (triObject) {
        _creator.add_tri_object(*triObject);
        if (triObject != object) {
          triObject->DeleteMe();
        }
      }
    }
  }

private:
  glTf_creator &_creator;
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

  auto extStr = path.extension().string();
  auto extStrLower = extStr;
  std::transform(extStrLower.begin(), extStrLower.end(), extStrLower.begin(),
                 ::tolower);

  glTf_creator creator;
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