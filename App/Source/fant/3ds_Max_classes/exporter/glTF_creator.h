
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
#include <variant>

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

  glTF_json serialize() const {
    glTF_json result;
    if (!_name.empty()) {
      result["name"] = _name;
    }
    return result;
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
    auto offset = _length;
    _length += size_;
    _buffers.emplace_back(size_);
    return {_buffers.back().data(), offset};
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

class image : public asset_base {
public:
  void source(std::u8string_view url_) {
    _source = std::u8string(url_.data(), url_.size());
  }

  void source(asset_ptr<buffer_view> buffer_view_) {
    _source = buffer_view_;
  }

  glTF_json serialize() const {
    auto result = asset_base::serialize();
    std::visit(
        [&](const auto &real_source_) {
          using RealSource = std::decay_t<decltype(real_source_)>;
          if constexpr (std::is_same_v<RealSource, std::u8string>) {
            result["uri"] = real_source_;
          } else if constexpr (std::is_same_v<RealSource,
                                              asset_ptr<buffer_view>>) {
            result["bufferView"] = 0;
          }
        },
        _source);
    return result;
  }

private:
  std::variant<std::monostate, std::u8string, asset_ptr<buffer_view>> _source;
};

class texture : public asset_base {
public:
private:
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
  asset_ptr<Asset> make(Args &&... args) {
    auto ptr = std::make_shared<Asset>(std::forward<Args>(args)...);
    std::get<_list_of_assets<Asset>>(_hub).push_back(ptr);
    return ptr;
  }

  template <typename Asset> asset_ptr<Asset> get(std::uint32_t index_) {
    return std::get<_list_of_assets<Asset>>(_hub)[index_];
  }

  template <typename Asset> std::uint32_t get_size() {
    return std::get<_list_of_assets<Asset>>(_hub).size();
  }

  void default_scene(asset_ptr<scene> scene_) {
    _defaultScene = scene_;
  }

  template <typename Asset> std::uint32_t index_of(asset_ptr<Asset> asset) {
    const auto &assets = std::get<_list_of_assets<Asset>>(_hub);
    if (auto r = std::find(assets.begin(), assets.end(), asset);
        r != assets.end()) {
      return std::distance(assets.begin(), r);
    }
    throw std::out_of_range;
  }

  glTF_json serialize() const {
    glTF_json result;
    return result;
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

using chunk_type_t = std::uint32_t;

inline constexpr chunk_type_t json_chunk = 0x4E4F534A;

inline constexpr std::byte json_chunk_padding_schema = std::byte(0x20);

inline constexpr chunk_type_t buffer_chunk = 0x4E4F534A;

inline constexpr std::byte buffer_chunk_padding_schema = std::byte(0);

struct chunk {
  chunk_type_t type;
  std::byte padding_schema;
  gsl::span<const std::byte> data;
};

chunk make_json_chunk(std::u8string_view gltf_json_str_) {
  return chunk{
      json_chunk, json_chunk_padding_schema,
      gsl::make_span(reinterpret_cast<const std::byte *>(gltf_json_str_.data()),
                     gltf_json_str_.size())};
}

namespace impl {
template <typename Int>
void write_little_endian(Int value_, std::byte *output_) {
  static_assert(std::endian::native == std::endian::little);
  *reinterpret_cast<Int *>(output_) = value_;
}
} // namespace impl

template <typename ChunkIterator>
std::vector<std::byte> write_glb(ChunkIterator first_chunk_,
                                 ChunkIterator last_chunk_) {
  std::uint32_t resultSize = 12;
  for (auto ichunk = first_chunk_; ichunk != last_chunk_; ++ichunk) {
    resultSize += ichunk->data.size();
    if (resultSize % 4 != 0) {
      resultSize = (resultSize / 4 + 1) * 4;
    }
  }

  std::vector<std::byte> result(resultSize);
  auto poutput = result.data();
  // Magic
  impl::write_little_endian(static_cast<std::uint32_t>(0x46546C67), poutput);
  poutput += 4;
  // Version
  impl::write_little_endian(static_cast<std::uint32_t>(2), poutput);
  poutput += 4;
  // Length
  impl::write_little_endian(static_cast<std::uint32_t>(resultSize), poutput);
  poutput += 4;
  for (auto ichunk = first_chunk_; ichunk != last_chunk_; ++ichunk) {
    // Chunk length
    impl::write_little_endian(static_cast<std::uint32_t>(ichunk->data.size()),
                              poutput);
    poutput += 4;
    // Chunk type
    impl::write_little_endian(static_cast<std::uint32_t>(ichunk->type),
                              poutput);
    poutput += 4;
    // Chunk data
    std::copy_n(ichunk->data.data(), ichunk->data.size(), poutput);
    poutput += ichunk->data.size();
    // Trailling padding
    if (auto nWritten = poutput - result.data(); nWritten % 4 != 0) {
      std::byte paddingByte = 0;
      if (ichunk->type == json_chunk) {
        paddingByte = 0x20;
      }
      auto nPadding = 4 - nWritten % 4;
      for (decltype(nWritten) iPadding = 0; iPadding < nPadding; ++iPadding) {
        *poutput++ = paddingByte;
      }
    }
  }

  return result;
}

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
} // namespace fant