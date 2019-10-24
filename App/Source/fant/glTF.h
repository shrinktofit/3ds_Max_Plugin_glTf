
#pragma once

#include <Max.h>
#include <cstdint>
#include <fant/3ds_Max_classes/export_settings.h>
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

using integer = std::uint64_t;

class document;

class animation;
class buffer;
class image;
class camera;
class texture;
class buffer_view;
class accessor;
class texture_sampler;
class mesh;
class material;
class skin;
class node;
class scene;

template <typename Object> using object_ptr = std::shared_ptr<Object>;

class extendable {
public:
  void add_extension(std::u8string_view key_, const glTF_json &value_) {
    _extensions.emplace(key_, value_);
  }

  void extras(const glTF_json &extras_) {
    _extras = extras_;
  }

protected:
  void serialize_extras_extensions(glTF_json &out_) const;

private:
  std::unordered_map<std::u8string, glTF_json> _extensions;
  std::optional<glTF_json> _extras;
};

class object_base : public extendable {
public:
  const std::u8string_view name() const {
    return _name;
  }

  void name(const std::u8string_view name_) {
    _name = name_;
  }

  glTF_json serialize(const document &document_) const;

private:
  std::u8string _name;
  std::unordered_map<std::u8string, glTF_json> _extensions;
};

template <typename... Objects> class object_factory {
public:
  template <typename Object, typename... Args>
  object_ptr<Object> make(Args &&... args) {
    auto ptr = std::make_shared<Object>(std::forward<Args>(args)...);
    std::get<_object_list<Object>>(_assetLists).push_back(ptr);
    return ptr;
  }

  template <typename Object> decltype(auto) get() {
    return std::get<_object_list<Object>>(_assetLists);
  }

  template <typename Object> decltype(auto) get() const {
    return std::get<_object_list<Object>>(_assetLists);
  }

  template <typename Object> object_ptr<Object> get(std::uint32_t index_) {
    return get<Object>()[index_];
  }

  template <typename Object> std::uint32_t get_size() {
    return static_cast<std::uint32_t>(get<Object>().size());
  }

  template <typename Object> integer index_of(object_ptr<Object> asset) const {
    const auto &assets = get<Object>();
    if (auto r = std::find(assets.begin(), assets.end(), asset);
        r != assets.end()) {
      return static_cast<integer>(std::distance(assets.begin(), r));
    }
    throw std::out_of_range("glTF object count out of range.");
  }

private:
  template <typename Asset> using _object_list = std::vector<object_ptr<Asset>>;

  std::tuple<_object_list<Objects>...> _assetLists;
};

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
  size_type _length = 0;
  std::list<_buffer_type> _buffers;
  std::optional<std::u8string> _uri;
};

class buffer_view : public object_base {
public:
  using size_type = buffer::size_type;

  struct allocate_result {
    allocate_result(object_ptr<glTF::buffer> buffer_,
                    size_type size_,
                    size_type alignment_)
        : buffer(buffer_), size(size_) {
      auto [d, s] = buffer_->allocate(size, alignment_);
      data = d;
      offset = s;
    }

    object_ptr<glTF::buffer> buffer;
    std::byte *data;
    size_type offset;
    size_type size;
  };

  enum class target_type {
    array_buffer = integer(34926),
    element_array_buffer = integer(34963),
  };

  buffer_view(allocate_result allocate_result_)
      : _allocateResult(allocate_result_) {
  }

  buffer_view(object_ptr<glTF::buffer> buffer_,
              size_type size_,
              size_type alignment_)
      : buffer_view(allocate_result(buffer_, size_, alignment_)) {
  }

  std::byte *data() {
    return _allocateResult.data;
  }

  void target(target_type target_) {
    _target = target_;
  }

  size_type stride() const {
    return _byteStride.value_or(0);
  }

  void stride(size_type stride_) {
    _byteStride = stride_;
  }

  size_type offset() const {
    return _allocateResult.offset;
  }

  glTF_json serialize(const document &document_) const;

private:
  allocate_result _allocateResult;
  std::optional<size_type> _byteStride;
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

  static size_type required_bytes(component_type component_) {
    switch (component_) {
    case component_type::the_byte:
    case component_type::unsigned_byte:
      return 1;
    case component_type::the_short:
    case component_type::unsigned_short:
      return 2;
    case component_type::unsigned_int:
    case component_type::the_float:
      return 4;
    }
    throw std::invalid_argument("");
  }

  template <component_type ComponentType> struct component_storage {};

  template <> struct component_storage<component_type::the_byte> {
    using type = std::int8_t;
  };

  template <> struct component_storage<component_type::unsigned_byte> {
    using type = std::uint8_t;
  };

  template <> struct component_storage<component_type::the_short> {
    using type = std::int16_t;
  };

  template <> struct component_storage<component_type::unsigned_short> {
    using type = std::uint16_t;
  };

  template <> struct component_storage<component_type::unsigned_int> {
    using type = std::uint32_t;
  };

  template <> struct component_storage<component_type::the_float> {
    using type = float;
  };

  template <component_type ComponentType>
  using component_storage_t = typename component_storage<ComponentType>::type;

  enum class type_type {
    scalar,
    vec2,
    vec3,
    vec4,
    mat2,
    mat3,
    mat4,
  };

  static size_type required_components(type_type type_) {
    switch (type_) {
    case type_type::scalar:
      return 1;
    case type_type::vec2:
      return 2;
    case type_type::vec3:
      return 3;
    case type_type::vec4:
      return 4;
    case type_type::mat2:
      return 4;
    case type_type::mat3:
      return 9;
    case type_type::mat4:
      return 16;
    }
    throw std::invalid_argument("");
  }

  accessor(object_ptr<glTF::buffer_view> buffer_view_,
           size_type offset_,
           component_type component_,
           type_type type_,
           size_type count_,
           bool explicit_bound_required_ = false)
      : _bufferView(buffer_view_), _byteOffset(offset_),
        _componentType(component_), _type(type_), _count(count_),
        _minMaxRequired(explicit_bound_required_) {
    assert((_byteOffset % required_bytes(_componentType) == 0) &&
           "The offset of an accessor into a bufferView must be a multiple of "
           "the size of the accessor's component type.");
    assert(((_byteOffset + buffer_view_->offset()) %
                required_bytes(_componentType) ==
            0) &&
           "The offset of an accessor into a buffer must be a multiple of the "
           "size of the accessor's component type.");
  }

  object_ptr<glTF::buffer_view> buffer_view() const {
    return _bufferView;
  }

  component_type component() const {
    return _componentType;
  }

  type_type type() const {
    return _type;
  }

  size_type count() const {
    return _count;
  }

  bool empty() const {
    return _count == 0;
  }

  auto data() const {
    return _bufferView->data() + _byteOffset;
  }

  auto data() {
    return _bufferView->data() + _byteOffset;
  }

  template <component_type Component> auto typed_data() {
    return reinterpret_cast<component_storage_t<Component> *>(
        _bufferView->data() + _byteOffset);
  }

  template <component_type Component> auto typed_data() const {
    return reinterpret_cast<const component_storage_t<Component> *>(
        _bufferView->data() + _byteOffset);
  }

  void explicit_bound_required(bool required_) {
    _minMaxRequired = required_;
  }

  glTF_json serialize(const document &document_) const;

private:
  object_ptr<glTF::buffer_view> _bufferView;
  size_type _byteOffset;
  component_type _componentType;
  type_type _type;
  size_type _count;
  bool _normalized = false;
  bool _minMaxRequired;
};

class image : public object_base {
public:
  enum class allowed_mime_type {
    jpeg,
    png,
  };

  void source(std::u8string_view url_) {
    _source = std::u8string(url_.data(), url_.size());
  }

  void source(object_ptr<buffer_view> buffer_view_,
              allowed_mime_type mime_type_) {
    _source = buffer_view_;
    _mimeType = mime_type_;
  }

  void source(gsl::span<std::byte> image_data_, allowed_mime_type mime_type_) {
    _source.emplace<std::vector<std::byte>>(image_data_.begin(),
                                            image_data_.end());
    _mimeType = mime_type_;
  }

  void mime_type(allowed_mime_type mime_type_) {
    _mimeType = mime_type_;
  }

  glTF_json serialize(const document &document_) const;

private:
  std::variant<std::monostate,
               std::u8string,
               object_ptr<buffer_view>,
               std::vector<std::byte>>
      _source;
  std::optional<allowed_mime_type> _mimeType;
};

class texture_sampler : public object_base {};

class texture : public object_base {
public:
  void source(glTF::object_ptr<glTF::image> image_) {
    _source = image_;
  }

  glTF_json serialize(const document &document_) const;

private:
  glTF::object_ptr<glTF::image> _source;
};

class texture_info : public extendable {
public:
  texture_info(object_ptr<glTF::texture> texture_) : _texture(texture_) {
  }

  glTF_json serialize(const document &document_) const;

private:
  constexpr inline static auto _defaultTexCoord = 0;

  object_ptr<glTF::texture> _texture;
  integer _texCoord = _defaultTexCoord;
};

class material : public object_base {
public:
  enum class alpha_mode_type {
    opaque,
    mask,
    blend,
  };

  class normal_textre_info : public texture_info {
  public:
    glTF_json serialize(const document &document_) const;

  private:
    constexpr inline static auto _defaultScale = 1;
    number _scale = _defaultScale;
  };

  class occlusion_textre_info : public texture_info {
  public:
    glTF_json serialize(const document &document_) const;

  private:
    constexpr inline static auto _defaultStrength = 1;
    number _stength = _defaultStrength;
  };

  class pbr_metallic_roughness_info : public extendable {
  public:
    void base_color_factor(number r_, number g_, number b_, number a_ = 1) {
      _baseColorFactor = {r_, g_, b_, a_};
    }

    void base_color_texture(texture_info info_) {
      _baseColorTexture = info_;
    }

    glTF_json serialize(const document &document_) const;

  private:
    constexpr inline static std::array<number, 4> _defaultBaseColorFactor = {
        1, 1, 1, 1};
    constexpr inline static auto _defaultMetallicFactor = 1;
    constexpr inline static auto _defaultRoughnessFactor = 1;

    std::array<number, 4> _baseColorFactor = _defaultBaseColorFactor;
    std::optional<texture_info> _baseColorTexture;
    number _metallicFactor = _defaultMetallicFactor;
    number _roughnessFactor = _defaultRoughnessFactor;
    std::optional<texture_info> _metallicRoughnessTexture;
  };

  void pbr_metallic_roughness(pbr_metallic_roughness_info value_) {
    _pbrMetallicRoughness = value_;
  }

  void emissive_factor(number f1_, number f2_, number f3_) {
    _emissiveFactor[0] = f1_;
    _emissiveFactor[1] = f2_;
    _emissiveFactor[2] = f3_;
  }

  void alpha_mode(alpha_mode_type value_) {
    _alphaMode = value_;
  }

  void alpha_cutoff(number value_) {
    _alphaCutoff = value_;
  }

  void double_sided(bool value_) {
    _doubleSided = value_;
  }

  glTF_json serialize(const document &document_) const;

private:
  constexpr inline static std::array<number, 3> _defaultEmissiveFactor = {0, 0,
                                                                          0};
  constexpr inline static auto _defaultAlphaMode = alpha_mode_type::opaque;
  constexpr inline static auto _defaultAlphaCutoff = 0.5;
  constexpr inline static auto _defaultDoubleSided = false;

  std::optional<pbr_metallic_roughness_info> _pbrMetallicRoughness;
  std::optional<normal_textre_info> _normalTexture;
  std::optional<occlusion_textre_info> _occlusionTexture;
  std::optional<texture_info> _emissiveTexture;
  std::array<number, 3> _emissiveFactor = {0, 0, 0};
  alpha_mode_type _alphaMode = _defaultAlphaMode;
  number _alphaCutoff = _defaultAlphaCutoff;
  bool _doubleSided = _defaultDoubleSided;
};

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
  return u8"WEIGHTS_" + to_u8string(set_);
}
} // namespace standard_semantics

class primitive : public extendable {
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

  void material(object_ptr<glTF::material> material_) {
    _material = material_;
  }

  glTF_json serialize(const document &document_) const;

private:
  mode_type _mode;
  std::unordered_map<std::u8string, object_ptr<accessor>> _attributes;
  object_ptr<accessor> _indices;
  object_ptr<glTF::material> _material;
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
  void add_joint(object_ptr<node> node_) {
    _joints.emplace_back(node_);
  }

  void inverse_bind_matrices(object_ptr<accessor> ibm_) {
    _inverseBindMatrices = ibm_;
  }

  glTF_json serialize(const document &document_) const;

private:
  std::vector<object_ptr<node>> _joints;
  object_ptr<accessor> _inverseBindMatrices;
};

class animation : public object_base {
public:
  class sampler : public extendable {
  public:
    sampler(object_ptr<accessor> input_, object_ptr<accessor> output_)
        : _input(input_), _output(output_) {
    }

    glTF_json serialize(const document &document_) const;

  private:
    object_ptr<accessor> _input;
    object_ptr<accessor> _output;
  };

  class channel : public extendable {
  public:
    class target_type : public extendable {
    public:
      enum class builtin_path {
        translation,
        rotation,
        scale,
        weights,
      };

      target_type(builtin_path path_) : _path(path_) {
      }

      target_type(builtin_path path_, object_ptr<glTF::node> node_)
          : _path(path_), _node(node_) {
      }

      void node(object_ptr<glTF::node> node_) {
        _node = node_;
      }

      glTF_json serialize(const document &document_) const;

    private:
      builtin_path _path;
      object_ptr<glTF::node> _node;
    };

    channel(object_ptr<sampler> sampler_, target_type target_)
        : _sampler(sampler_), _target(target_) {
    }

    target_type target() {
      return _target;
    }

    glTF_json serialize(const document &document_,
                        const animation &animation_) const;

  private:
    object_ptr<sampler> _sampler;
    target_type _target;
  };

  using factory_type = object_factory<sampler, channel>;

  const factory_type &factory() const {
    return _factory;
  }

  factory_type &factory() {
    return _factory;
  }

  glTF_json serialize(const document &document_) const;

private:
  factory_type _factory;
};

class node : public object_base, public std::enable_shared_from_this<node> {
public:
  using position_type = std::array<number, 3>;

  using scale_type = std::array<number, 3>;

  using rotation_type = std::array<number, 4>;

  using matrix_type = std::array<number, 16>;

  object_ptr<node> parent() {
    return _parent.expired() ? nullptr : _parent.lock();
  }

  bool is_trs() const {
    return std::holds_alternative<_trs_type>(_transform);
  }

  position_type position() const {
    return std::get<_trs_type>(_transform).position;
  }

  void position(const position_type &value_) {
    std::get<_trs_type>(_transform).position = value_;
  }

  scale_type scale() const {
    return std::get<_trs_type>(_transform).scale;
  }

  void scale(const scale_type &value_) {
    std::get<_trs_type>(_transform).scale = value_;
  }

  rotation_type rotation() const {
    return std::get<_trs_type>(_transform).rotation;
  }

  matrix_type matrix() const {
    return std::get<matrix_type>(_transform);
  }

  void rotation(const rotation_type &value_) {
    std::get<_trs_type>(_transform).rotation = value_;
  }

  void mesh(object_ptr<glTF::mesh> mesh_) {
    _mesh = mesh_;
  }

  void skin(object_ptr<glTF::skin> skin_) {
    _skin = skin_;
  }

  void add_child(object_ptr<glTF::node> child_) {
    assert(child_->_parent.expired());
    _children.push_back(child_);
    child_->_parent = shared_from_this();
  }

  glTF_json serialize(const document &document_) const;

private:
  std::weak_ptr<node> _parent;
  std::vector<object_ptr<node>> _children;
  object_ptr<glTF::mesh> _mesh;
  object_ptr<glTF::skin> _skin;
  struct _trs_type {
    std::array<number, 3> position = {(number)0, (number)0, (number)0};
    std::array<number, 3> scale = {(number)1, (number)1, (number)1};
    std::array<number, 4> rotation = {(number)0, (number)0, (number)0,
                                      (number)1};
  };
  std::variant<_trs_type, matrix_type> _transform;
};

class camera : public object_base {};

class scene : public object_base {
public:
  void add_node(object_ptr<node> node_) {
    _nodes.push_back(node_);
  }

  glTF_json serialize(const document &document_) const;

private:
  std::vector<object_ptr<node>> _nodes;
};

class document : public extendable {
public:
  using factory_type = object_factory<animation,
                                      buffer,
                                      image,
                                      camera,
                                      texture,
                                      buffer_view,
                                      accessor,
                                      texture_sampler,
                                      mesh,
                                      material,
                                      skin,
                                      node,
                                      scene>;

  void default_scene(object_ptr<scene> scene_) {
    _defaultScene = scene_;
  }

  glTF_json serialize(bool rm_uri_of_first_buffer_) const;

  const factory_type &factory() const {
    return _factory;
  }

  factory_type &factory() {
    return _factory;
  }

private:
  factory_type _factory;
  object_ptr<scene> _defaultScene;
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