
#pragma once

#include <Max.h>
#include <cstdint>
#include <fant/3ds_Max_classes/exporter/export_settings.h>
#include <fant/3ds_Max_classes/exporter/vertex_list.h>
#include <fant/support/win32/mchar_to_utf8.h>
#include <filesystem>
#include <fx/gltf.h>
#include <gsl/span>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>

std::string to_glTF_string(std::u8string_view string_) {
  return std::string(reinterpret_cast<const char *>(string_.data()),
                     string_.size());
}

class glTf_overflow : public std::exception {
public:
  enum class target_type {
    accessor_count,
    buffer_count,
    buffer_view_count,
    mesh_count,
    node_count,
    scene_count,

    buffer,
  };

  glTf_overflow(target_type target_) : _target(target_) {
  }

  target_type target() const {
    return _target;
  }

private:
  target_type _target;
};

template <class T> struct dependent_false : std::false_type {};

namespace fant {
enum class vertex_semantic {
  position,
  normal,
  texcoord,
  color,
};

namespace glTF {
using glTF_json = nlohmann::basic_json<std::map, std::vector, std::u8string>;

using number = std::uint32_t;

class asset_base {
public:
  void name(const std::u8string_view name_) {
    _name = name_;
  }

private:
  std::u8string _name;
  std::unordered_map<std::u8string, glTF_json> _extensions;
};

template <typename Asset> using asset_ptr = std::shared_ptr<Asset>;
class buffer : public asset_base {
public:
  using size_type = std::uint32_t;

  size_type size() const {
    return _length;
  }

  std::pair<std::byte *, size_type> allocate(size_type size_,
                                             size_type alignment_) {
    if (alignment_ != 0) {
      if (auto remainder = _length % alignment_; remainder != 0) {
        auto complement = alignment_ - remainder;
        _length += complement;
        _buffers.emplace_back(complement);
      }
    }
    fx::gltf::BufferView bufferView;
    bufferView.buffer = _index;
    bufferView.byteOffset = _length;
    bufferView.byteLength = size_;
    bufferView.byteStride = 0;
    _length += size_;
    _buffers.emplace_back(size_);
    return {_buffers.back().data(), bufferView };
  }

  void read_all(std::byte *output_) {
    for (auto &buffer : _buffers) {
      std::copy(buffer.begin(), buffer.end(), output_);
      output_ += buffer.size();
    }
  }

private:
  using _buffer_type = std::vector<std::byte>;
  std::uint32_t _index;
  std::uint32_t _length = 0;
  std::list<_buffer_type> _buffers;
};

class buffer_view : public asset_base {
public:
  using size_type = buffer::size_type;

  struct allocate_result {
    allocate_result(asset_ptr<glTF::buffer> buffer_,
                    size_type size_,
                    size_type alignment_)
        : buffer(buffer_), size(size_) {
      auto [d, s] = buffer_->allocate(size, alignment_);
      data = d;
      offset = s;
    }

    asset_ptr<glTF::buffer> buffer;
    std::byte *data;
    size_type offset;
    size_type size;
  };

  enum class target_type {
    element_array_buffer,
  };

  buffer_view(allocate_result allocate_result_, size_type byte_stride_)
      : _allocateResult(allocate_result_), _byteStride(byte_stride_) {
  }

  buffer_view(asset_ptr<glTF::buffer> buffer_,
              size_type size_,
              size_type alignment_,
              size_type byte_stride_)
      : buffer_view(allocate_result(buffer_, size_, alignment_), byte_stride_) {
  }

  std::byte *data() {
    return _allocateResult.data;
  }

  void target(target_type target_) {
    _target = target_;
  }

private:
  allocate_result _allocateResult;
  size_type _byteStride;
  std::optional<target_type> _target;
};

class accessor : public asset_base {
public:
  using size_type = buffer::size_type;

  enum class component_type {
    the_byte,
    unsigned_byte,
    the_short,
    unsigned_short,
    unsigned_int,
    the_float,
  };

  enum class type_type {
    scalar,
    vec2,
    vec3,
    vec4,
    mat2,
    mat3,
    mat4,
  };

  accessor(asset_ptr<glTF::buffer_view> buffer_view_,
           size_type offset_,
           component_type component_,
           type_type type_,
           size_type count_)
      : _bufferView(buffer_view_), _offset(offset_), _componentType(component_),
        _type(type_), _count(count_) {
  }

private:
  asset_ptr<glTF::buffer_view> _bufferView;
  size_type _offset;
  component_type _componentType;
  type_type _type;
  size_type _count;
};

class material : public asset_base {};

class primitive : public asset_base {
public:
  enum class semantic_type {
    position,
    normal,
    tangent,
    texcoord_0,
    texcoord_1,
    color_0,
    joints_0,
    weights_0,
  };

  enum class mode_type {
    points,
    lines,
    line_loop,
    line_strip,
    triangles,
    triangle_strip,
    triangle_fan,
  };

  primitive(mode_type mode_) : _mode(mode_) {
  }

  void emplace_attribute(semantic_type semantic_, asset_ptr<accessor> data_) {
    _attributes.emplace(semantic_, data_);
  }

  void indices(asset_ptr<accessor> indices_) {
    _indices = indices_;
  }

private:
  mode_type _mode;
  std::unordered_map<semantic_type, asset_ptr<accessor>> _attributes;
  asset_ptr<accessor> _indices;
  asset_ptr<material> _material;
};

class mesh : public asset_base {
public:
  void push_primitive(const primitive &primitive_) {
    _primitives.push_back(primitive_);
  }

private:
  std::vector<primitive> _primitives;
};

class skin : public asset_base {
public:
private:
};

class animation : public asset_base {};

class node : public asset_base {
public:
  void mesh(asset_ptr<glTF::mesh> mesh_) {
    _mesh = mesh_;
  }

  void material(asset_ptr<glTF::material> material_) {
    _material = material_;
  }

  void add_child(asset_ptr<glTF::node> child_) {
    _children.push_back(child_);
  }

  void set_position(number x_, number y_, number z_) {
    _position.emplace();
    (*_position)[0] = x_;
    (*_position)[1] = y_;
    (*_position)[2] = z_;
  }

  void set_scale(number x_, number y_, number z_) {
    _scale.emplace();
    (*_scale)[0] = x_;
    (*_scale)[1] = y_;
    (*_scale)[2] = z_;
  }

  void set_rotation(number x_, number y_, number z_, number w_) {
    _scale.emplace();
    (*_rotation)[0] = x_;
    (*_rotation)[1] = y_;
    (*_rotation)[2] = z_;
    (*_rotation)[3] = w_;
  }

private:
  std::vector<asset_ptr<node>> _children;
  asset_ptr<glTF::mesh> _mesh;
  asset_ptr<glTF::material> _material;
  std::optional<std::array<number, 3>> _position;
  std::optional<std::array<number, 3>> _scale;
  std::optional<std::array<number, 4>> _rotation;
  std::optional<std::array<number, 16>> _matrix;
};

class scene : public asset_base {
public:
  void add_node(asset_ptr<node> node_) {
    _nodes.push_back(node_);
  }

private:
  std::vector<asset_ptr<node>> _nodes;
};

class document {
public:
  template <typename Asset, typename... Args>
  asset_ptr<Asset> make(Args&& ... args) {
    auto ptr = std::make_shared<Asset>(std::forward<Args>(args)...);
    std::get<_list_of_assets<Asset>>(_hub).push_back(ptr);
    return ptr;
  }

  void default_scene(asset_ptr<scene> scene_) {
    _defaultScene = scene_;
  }

private:
  asset_ptr<scene> _defaultScene;

  template <typename Asset>
  using _list_of_assets = std::vector<asset_ptr<Asset>>;

  std::tuple<_list_of_assets<buffer>,
    _list_of_assets<buffer_view>,
    _list_of_assets<accessor>,
    _list_of_assets<mesh>,
    _list_of_assets<material>,
    _list_of_assets<skin>,
    _list_of_assets<node>,
    _list_of_assets<scene>>
    _hub;
};

} // namespace glTF

class vertex_list2 {
public:
  template <typename Ty>
  void add(vertex_semantic semantic_, gsl::span<Ty> data_) {
    auto data = std::make_unique<std::byte[]>(data_.size_bytes());
    std::copy(reinterpret_cast<const std::byte *>(data_.data()),
              data_.size_bytes(), data.get());
    _attributes.emplace(semantic_, std::move(data));
  }

private:
  std::unordered_map<vertex_semantic, std::unique_ptr<std::byte[]>> _attributes;
};

class glTf_creator {
public:
  using node_handle = std::uint32_t;

  using mesh_handle = std::uint32_t;

  using animation_handle = std::uint32_t;

  using scene_handle = std::uint32_t;

  glTf_creator(const export_settings &settings_, Interface &max_interface_)
      : _settings(settings_), _maxInterface(max_interface_) {
  }

  glTF::document &document() {
    return _document;
  }

  void commit() {
    if (_bufferSegments.length() == 0) {
      (void)_bufferSegments.allocate_view(1, 1);
    }
    _glTfDocument.buffers.emplace_back();
    auto &mainBuffer = _glTfDocument.buffers.back();
    mainBuffer.data.resize(_bufferSegments.length());
    _bufferSegments.read_all(
        reinterpret_cast<std::byte *>(mainBuffer.data.data()));
    mainBuffer.byteLength = _bufferSegments.length();
  }

  /// <summary>
  /// Save as .glb format.
  /// </summary>
  /// <param name="path_"></param>
  /// <param name="binary_dir_name_"></param>
  /// <param name="binary_ext_"></param>
  void save(std::u8string_view path_,
            std::u8string_view binary_dir_name_,
            std::u8string_view binary_ext_) {
    for (decltype(_glTfDocument.buffers)::size_type iBuffer = 0;
         iBuffer != _glTfDocument.buffers.size(); ++iBuffer) {
      auto &buffer = _glTfDocument.buffers[iBuffer];
      std::basic_stringstream<char8_t> urlss;
      urlss << u8"./" << binary_dir_name_;
      if (_glTfDocument.buffers.size() != 1) {
        std::array<char, sizeof(iBuffer) * CHAR_BIT> indexChars;
        auto result = std::to_chars(
            indexChars.data(), indexChars.data() + indexChars.size(), iBuffer);
        assert(result.ec == std::errc());
        // TODO: indexString is not guarenteen as UTF8.
        auto indexString = std::u8string_view(
            reinterpret_cast<const char8_t *>(indexChars.data()),
            result.ptr - indexChars.data());
        urlss << u8"-" << indexString;
      }
      urlss << binary_ext_;
      buffer.uri =
          to_glTF_string(std::filesystem::path(urlss.str()).u8string());
    }
    _doSave(path_, false);
  }

  /// <summary>
  /// Save as .gltf file.
  /// </summary>
  /// <param name="path_"></param>
  void save(std::u8string_view path_) {
    for (auto &buffer : _glTfDocument.buffers) {
      buffer.uri.clear();
    }
    _doSave(path_, true);
  }

private:
  using _buffer_type = std::vector<std::byte>;

  glTF::document _document;
  fx::gltf::Document _glTfDocument;
  const export_settings &_settings;
  Interface &_maxInterface;

  void _doSave(std::u8string_view path_, bool binary_) try {
    auto path = std::filesystem::path(path_).string();
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

  std::uint32_t _addScene(fx::gltf::Scene &&scene_) {
    auto result = _glTfDocument.scenes.size();
    _glTfDocument.scenes.push_back(std::move(scene_));
    return _ensureAssetCount(glTf_overflow::target_type::scene_count, result);
  }

  std::uint32_t _addNode(fx::gltf::Node &&node_) {
    auto result = _glTfDocument.nodes.size();
    _glTfDocument.nodes.push_back(std::move(node_));
    return _ensureAssetCount(glTf_overflow::target_type::node_count, result);
  }

  std::uint32_t _addMesh(fx::gltf::Mesh &&mesh_) {
    auto result = _glTfDocument.meshes.size();
    _glTfDocument.meshes.push_back(std::move(mesh_));
    return _ensureAssetCount(glTf_overflow::target_type::mesh_count, result);
  }

  std::uint32_t _addBufferView(fx::gltf::BufferView &&buffer_view_) {
    auto result = _glTfDocument.bufferViews.size();
    _glTfDocument.bufferViews.push_back(std::move(buffer_view_));
    return _ensureAssetCount(glTf_overflow::target_type::buffer_view_count,
                             result);
  }

  std::uint32_t _addAccessor(fx::gltf::Accessor &&accessor_) {
    auto result = _glTfDocument.accessors.size();
    _glTfDocument.accessors.push_back(std::move(accessor_));
    return _ensureAssetCount(glTf_overflow::target_type::accessor_count,
                             result);
  }

  template <typename UInt>
  std::uint32_t _ensureAssetCount(glTf_overflow::target_type target_,
                                  UInt count_) {
    return _ensureNotOverflow<std::uint32_t>(target_, count_);
  }

  template <typename CapacityInt, typename UInt>
  std::uint32_t _ensureNotOverflow(glTf_overflow::target_type target_,
                                   UInt count_) {
    if (count_ > std::numeric_limits<CapacityInt>::max()) {
      throw glTf_overflow(target_);
    }
    return static_cast<CapacityInt>(count_);
  }
};
} // namespace fant