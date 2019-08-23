
#pragma once

#include <Max.h>
#include <cstdint>
#include <fant/3ds_Max_classes/exporter/export_settings.h>
#include <fant/3ds_Max_classes/exporter/vertex_list.h>
#include <fant/support/to_u8string.h>
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
using glTF_json = nlohmann::json;

using number = float;

using integer = std::uint32_t;

class document;

class object_base {
public:
  void name(const std::u8string_view name_) {
    _name = name_;
  }

  glTF_json serialize(const document &document_) const;

private:
  std::u8string _name;
  std::unordered_map<std::u8string, glTF_json> _extensions;
};

template <typename Asset> using object_ptr = std::shared_ptr<Asset>;
class buffer : public object_base {
public:
  using size_type = std::uint32_t;

  size_type size() const {
    return _length;
  }

  void uri(std::u8string_view uri_) {
    _uri = uri_;
  }

  std::optional<std::u8string_view> uri() {
    return _uri;
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

  void read_all(std::byte *output_) const {
    for (auto &buffer : _buffers) {
      std::copy(buffer.begin(), buffer.end(), output_);
      output_ += buffer.size();
    }
  }

  glTF_json serialize(const document &document_, bool write_uri_) const;

private:
  using _buffer_type = std::vector<std::byte>;
  std::uint32_t _index;
  std::uint32_t _length = 0;
  std::list<_buffer_type> _buffers;
  std::optional<std::u8string> _uri;
};

class buffer_view : public object_base {
public:
  using size_type = buffer::size_type;

  struct allocate_result {
    allocate_result(object_ptr<glTF::buffer> buffer_,
                    integer size_,
                    integer alignment_)
        : buffer(buffer_), size(size_) {
      auto [d, s] = buffer_->allocate(size, alignment_);
      data = d;
      offset = s;
    }

    object_ptr<glTF::buffer> buffer;
    std::byte *data;
    integer offset;
    integer size;
  };

  enum class target_type {
    array_buffer = integer(34926),
    element_array_buffer = integer(34963),
  };

  buffer_view(allocate_result allocate_result_)
      : _allocateResult(allocate_result_) {
  }

  buffer_view(object_ptr<glTF::buffer> buffer_,
              integer size_,
              integer alignment_)
      : buffer_view(allocate_result(buffer_, size_, alignment_)) {
  }

  std::byte *data() {
    return _allocateResult.data;
  }

  void target(target_type target_) {
    _target = target_;
  }

  void stride(integer stride_) {
    _byteStride = stride_;
  }

  glTF_json serialize(const document &document_) const;

private:
  allocate_result _allocateResult;
  std::optional<integer> _byteStride;
  std::optional<target_type> _target;
};

class accessor : public object_base {
public:
  using size_type = buffer::size_type;

  enum class component_type {
    the_byte = integer(5120),
    unsigned_byte = integer(5121),
    the_short = integer(5122),
    unsigned_short = integer(5123),
    unsigned_int = integer(5125),
    the_float = integer(5126),
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

  accessor(object_ptr<glTF::buffer_view> buffer_view_,
           size_type offset_,
           component_type component_,
           type_type type_,
           size_type count_)
      : _bufferView(buffer_view_), _byteOffset(offset_),
        _componentType(component_), _type(type_), _count(count_) {
  }

  glTF_json serialize(const document &document_) const;

private:
  object_ptr<glTF::buffer_view> _bufferView;
  size_type _byteOffset;
  component_type _componentType;
  type_type _type;
  integer _count;
  bool _normalized = false;
};

class image : public object_base {
public:
  void source(std::u8string_view url_) {
    _source = std::u8string(url_.data(), url_.size());
  }

  void source(object_ptr<buffer_view> buffer_view_,
              std::u8string_view mime_type_) {
    _source = buffer_view_;
    _mimeType = mime_type_;
  }

  void mime_type(std::u8string_view mime_type_) {
    _mimeType = mime_type_;
  }

  glTF_json serialize(const document &document_) const;

private:
  std::variant<std::monostate, std::u8string, object_ptr<buffer_view>> _source;
  std::optional<std::u8string> _mimeType;
};

class texture_sampler : public object_base {};

class texture : public object_base {
public:
private:
};

class material : public object_base {};

namespace standard_semantics {
inline constexpr auto position = u8"POSITION";

inline constexpr auto normal = u8"NORMAL";

inline constexpr auto tangent = u8"TANGENT";

inline std::u8string texcoord(unsigned set_) {
  return u8"TEXCOORD_" + to_u8string(set_);
}

inline std::u8string color(unsigned set_) {
  return u8"COLOR_" + to_u8string(set_);
}

inline std::u8string joints(unsigned set_) {
  return u8"JOINTS_" + to_u8string(set_);
}

inline std::u8string weights(unsigned set_) {
  return u8"WEIGHTS" + to_u8string(set_);
}
} // namespace standard_semantics

class primitive {
public:
  enum class mode_type {
    points = integer(0),
    lines = integer(1),
    line_loop = integer(2),
    line_strip = integer(3),
    triangles = integer(4),
    triangle_strip = integer(5),
    triangle_fan = integer(6),
  };

  primitive(mode_type mode_) : _mode(mode_) {
  }

  void emplace_attribute(std::u8string_view name_, object_ptr<accessor> data_) {
    _attributes.emplace(name_, data_);
  }

  void indices(object_ptr<accessor> indices_) {
    _indices = indices_;
  }

  glTF_json serialize(const document &document_) const;

private:
  mode_type _mode;
  std::unordered_map<std::u8string, object_ptr<accessor>> _attributes;
  object_ptr<accessor> _indices;
  object_ptr<material> _material;
};

class mesh : public object_base {
public:
  void push_primitive(const primitive &primitive_) {
    _primitives.push_back(primitive_);
  }

  glTF_json serialize(const document &document_) const;

private:
  std::vector<primitive> _primitives;
};

class skin : public object_base {
public:
private:
};

class animation : public object_base {};

class node : public object_base {
public:
  void mesh(object_ptr<glTF::mesh> mesh_) {
    _mesh = mesh_;
  }

  void material(object_ptr<glTF::material> material_) {
    _material = material_;
  }

  void add_child(object_ptr<glTF::node> child_) {
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
  std::vector<object_ptr<node>> _children;
  object_ptr<glTF::mesh> _mesh;
  object_ptr<glTF::material> _material;
  std::optional<std::array<number, 3>> _position;
  std::optional<std::array<number, 3>> _scale;
  std::optional<std::array<number, 4>> _rotation;
  std::optional<std::array<number, 16>> _matrix;
};

class camera : public object_base {};

class scene : public object_base {
public:
  void add_node(object_ptr<node> node_) {
    _nodes.push_back(node_);
  }

private:
  std::vector<object_ptr<node>> _nodes;
};

class document {
public:
  template <typename Asset, typename... Args>
  object_ptr<Asset> make(Args &&... args) {
    auto ptr = std::make_shared<Asset>(std::forward<Args>(args)...);
    std::get<_object_list<Asset>>(_assetLists).push_back(ptr);
    return ptr;
  }

  template <typename Asset> object_ptr<Asset> get(std::uint32_t index_) {
    return std::get<_object_list<Asset>>(_assetLists)[index_];
  }

  template <typename Asset> std::uint32_t get_size() {
    return static_cast<std::uint32_t>(
        std::get<_object_list<Asset>>(_assetLists).size());
  }

  void default_scene(object_ptr<scene> scene_) {
    _defaultScene = scene_;
  }

  template <typename Asset> integer index_of(object_ptr<Asset> asset) const {
    const auto &assets = std::get<_object_list<Asset>>(_assetLists);
    if (auto r = std::find(assets.begin(), assets.end(), asset);
        r != assets.end()) {
      return static_cast<integer>(std::distance(assets.begin(), r));
    }
    throw std::out_of_range("glTF object count out of range.");
  }

  glTF_json serialize(bool rm_uri_of_first_buffer_) const;

private:
  object_ptr<scene> _defaultScene;

  template <typename Asset> using _object_list = std::vector<object_ptr<Asset>>;

  std::tuple<_object_list<animation>,
             _object_list<buffer>,
             _object_list<image>,
             _object_list<camera>,
             _object_list<texture>,
             _object_list<buffer_view>,
             _object_list<accessor>,
             _object_list<texture_sampler>,
             _object_list<mesh>,
             _object_list<material>,
             _object_list<skin>,
             _object_list<node>,
             _object_list<scene>>
      _assetLists;
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

inline chunk make_json_chunk(
    std::basic_string_view<glTF_json::string_t::value_type> gltf_json_str_) {
  return chunk{
      json_chunk, json_chunk_padding_schema,
      gsl::make_span(reinterpret_cast<const std::byte *>(gltf_json_str_.data()),
                     gltf_json_str_.size())};
}

inline chunk make_buffer_chunk(gsl::span<std::byte> data_) {
  return chunk{buffer_chunk, buffer_chunk_padding_schema, data_};
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
    resultSize += static_cast<std::uint32_t>(ichunk->data.size());
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
      std::byte paddingByte = std::byte(0);
      if (ichunk->type == json_chunk) {
        paddingByte = std::byte(0x20);
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