
#include <cppcodec/base64_url.hpp>
#include <fant/glTF.h>

namespace fant::glTF {
static std::string to_json_string(std::u8string_view string_) {
  return std::string(reinterpret_cast<const char *>(string_.data()),
                     string_.size());
}

glTF_json object_base::serialize(const document &document_) const {
  glTF_json result;
  if (!_name.empty()) {
    result["name"] = to_json_string(_name);
  }
  return result;
}

glTF_json buffer::serialize(const document &document_, bool write_uri_) const {
  auto result = object_base::serialize(document_);
  if (write_uri_) {
    if (_uri) {
      result["uri"] = to_json_string(*_uri);
    } else {
      std::vector<std::byte> data(size());
      read_all(data.data());
      // Unsafe
      static_assert(std::is_same_v<std::uint8_t, unsigned char>);
      auto datauri = cppcodec::base64_url::encode(
          reinterpret_cast<const std::uint8_t *>(data.data()), data.size());
      result["uri"] = "data:application/octet-stream;base64," + datauri;
    }
  }
  return result;
}

glTF_json accessor::serialize(const document &document_) const {
  auto result = object_base::serialize(document_);
  if (_bufferView) {
    result["bufferView"] = document_.index_of(_bufferView);
  }
  if (_byteOffset) {
    result["byteOffset"] = _byteOffset;
  }
  result["componentType"] = static_cast<integer>(_componentType);
  if (_normalized) {
    result["normalized"] = _normalized;
  }
  result["count"] = _count;
  auto serializeType = [this]() {
    switch (_type) {
    case type_type::scalar:
      return "SCALAR";
    case type_type::vec2:
      return "VEC2";
    case type_type::vec3:
      return "VEC3";
    case type_type::vec4:
      return "VEC4";
    case type_type::mat2:
      return "MAT2";
    case type_type::mat3:
      return "MAT3";
    case type_type::mat4:
      return "MAT4";
    default:
      return "";
    }
  };
  result["type"] = serializeType();
  return result;
}

glTF_json buffer_view::serialize(const document &document_) const {
  auto result = object_base::serialize(document_);
  result["buffer"] = document_.index_of(_allocateResult.buffer);
  if (_allocateResult.offset != 0) {
    result["byteOffset"] = _allocateResult.offset;
  }
  result["byteLength"] = _allocateResult.size;
  if (_byteStride.has_value()) {
    result["byteStride"] = *_byteStride;
  }
  if (_target.has_value()) {
    result["target"] = static_cast<integer>(*_target);
  }
  return result;
}

glTF_json mesh::serialize(const document &document_) const {
  auto result = object_base::serialize(document_);
  glTF_json primitivesJson;
  for (auto &primitive : _primitives) {
    primitivesJson.push_back(primitive.serialize(document_));
  }
  return result;
}

glTF_json primitive::serialize(const document &document_) const {
  glTF_json result;
  glTF_json attributesJson;
  for (auto &attribute : _attributes) {
    attributesJson[to_json_string(attribute.first)] =
        document_.index_of(attribute.second);
  }
  result["attributes"] = attributesJson;
  if (_indices) {
    result["indices"] = document_.index_of(_indices);
  }
  if (_material) {
    result["material"] = document_.index_of(_material);
  }
  if (_mode != mode_type::triangles) {
    result["mode"] = static_cast<integer>(_mode);
  }
  return result;
}

glTF_json image::serialize(const document &document_) const {
  auto result = object_base::serialize(document_);
  std::visit(
      [&](const auto &real_source_) {
        using RealSource = std::decay_t<decltype(real_source_)>;
        if constexpr (std::is_same_v<RealSource, std::u8string>) {
          result["uri"] = to_json_string(real_source_);
        } else if constexpr (std::is_same_v<RealSource,
                                            object_ptr<buffer_view>>) {
          result["bufferView"] = document_.index_of(real_source_);
        }
      },
      _source);
  if (_mimeType) {
    result["mimeType"] = to_json_string(*_mimeType);
  }
  return result;
}

template <typename Object> struct object_type_t { using type = Object; };

template <typename Object>
inline constexpr static object_type_t<Object> object_type = {};

glTF_json document::serialize(bool rm_uri_of_first_buffer_) const {
  glTF_json result;

  auto serializeObjectArray = [&](auto assetType) {
    using AssetType = typename std::decay_t<decltype(assetType)>::type;
    glTF_json arrayJson;
    auto &assetArray = std::get<_object_list<AssetType>>(_assetLists);
    for (auto i = assetArray.begin(); i != assetArray.end(); ++i) {
      auto &asset = *i;
      if constexpr (std::is_same_v<AssetType, buffer>) {
        auto rmuri = i == assetArray.begin() && rm_uri_of_first_buffer_;
        arrayJson.push_back(asset->serialize(*this, rmuri));
      } else {
        arrayJson.push_back(asset->serialize(*this));
      }
    }
    return arrayJson;
  };

  result["accessors"] = serializeObjectArray(object_type<accessor>);
  result["animations"] = serializeObjectArray(object_type<animation>);
  result["buffers"] = serializeObjectArray(object_type<buffer>);
  result["bufferViews"] = serializeObjectArray(object_type<buffer_view>);
  result["cameras"] = serializeObjectArray(object_type<camera>);
  result["images"] = serializeObjectArray(object_type<image>);
  result["materials"] = serializeObjectArray(object_type<material>);
  result["meshes"] = serializeObjectArray(object_type<mesh>);
  result["nodes"] = serializeObjectArray(object_type<node>);
  result["samplers"] = serializeObjectArray(object_type<texture_sampler>);
  result["scenes"] = serializeObjectArray(object_type<scene>);
  result["skins"] = serializeObjectArray(object_type<skin>);
  result["textures"] = serializeObjectArray(object_type<texture>);

  return result;
}
} // namespace fant::glTF
