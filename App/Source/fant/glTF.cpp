
#include <cppcodec/base64_rfc4648.hpp>
#include <fant/glTF.h>

namespace fant::glTF {
static std::string to_json_string(std::u8string_view string_) {
  return std::string(reinterpret_cast<const char *>(string_.data()),
                     string_.size());
}

void extendable::serialize_extras_extensions(glTF_json &out_) const {
  if (!_extensions.empty()) {
    out_["extensions"] = _extensions;
  }
  if (_extras) {
    out_["extras"] = *_extras;
  }
}

glTF_json object_base::serialize(const document &document_) const {
  auto result = glTF_json::object();
  if (!_name.empty()) {
    result["name"] = to_json_string(_name);
  }
  serialize_extras_extensions(result);
  return result;
}

glTF_json buffer::serialize(const document &document_, bool write_uri_) const {
  auto result = object_base::serialize(document_);
  result["byteLength"] = size();
  if (write_uri_) {
    if (_uri) {
      result["uri"] = to_json_string(*_uri);
    } else {
      std::vector<std::byte> data(size());
      read_all(data.data());
      // Unsafe
      static_assert(std::is_same_v<std::uint8_t, unsigned char>);
      auto datauri = cppcodec::base64_rfc4648::encode(
          reinterpret_cast<const std::uint8_t *>(data.data()), data.size());
      result["uri"] = "data:application/octet-stream;base64," + datauri;
    }
  }
  return result;
}

template <accessor::component_type ComponentType>
static void eval_min_max(const accessor &accessor_, glTF_json &target_) {
  if (accessor_.empty()) {
    return;
  }

  unsigned nComp = 0;
  switch (accessor_.type()) {
  case accessor::type_type::scalar:
    nComp = 1;
    break;
  case accessor::type_type::vec2:
    nComp = 2;
    break;
  case accessor::type_type::vec3:
    nComp = 3;
    break;
  case accessor::type_type::vec4:
    nComp = 4;
    break;
  case accessor::type_type::mat2:
    nComp = 4;
    break;
  case accessor::type_type::mat3:
    nComp = 9;
    break;
  case accessor::type_type::mat4:
    nComp = 16;
    break;
  }

  using StorageType = accessor::component_storage_t<ComponentType>;

  auto pdata = accessor_.buffer_view()->data();
  auto stride = accessor_.buffer_view()->stride();
  if (stride == 0) {
    stride = sizeof(StorageType) * nComp;
  }

  StorageType minbuffer[16] = {};
  StorageType maxbuffer[16] = {};

  // Write first.
  std::copy_n(reinterpret_cast<const StorageType *>(pdata), nComp, minbuffer);
  std::copy_n(reinterpret_cast<const StorageType *>(pdata), nComp, maxbuffer);
  pdata += stride;

  for (std::remove_const_t<decltype(accessor_.count())> iAttribute = 1;
       iAttribute < accessor_.count(); ++iAttribute, pdata += stride) {
    auto pFirstComp = reinterpret_cast<const StorageType *>(pdata);
    for (std::remove_const_t<decltype(nComp)> iComp = 0; iComp < nComp;
         ++iComp) {
      auto comp = pFirstComp[iComp];
      auto &minc = minbuffer[iComp];
      minc = std::min(minc, comp);
      auto &maxc = maxbuffer[iComp];
      maxc = std::max(maxc, comp);
    }
  }

  auto minJson = glTF_json::array();
  auto maxJson = glTF_json::array();
  for (std::remove_const_t<decltype(nComp)> iComp = 0; iComp < nComp; ++iComp) {
    minJson.push_back(minbuffer[iComp]);
    maxJson.push_back(maxbuffer[iComp]);
  }
  target_["min"] = minJson;
  target_["max"] = maxJson;
}

glTF_json accessor::serialize(const document &document_) const {
  auto result = object_base::serialize(document_);
  if (_bufferView) {
    result["bufferView"] = document_.factory().index_of(_bufferView);
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

  if (_minMaxRequired) {
    std::array<std::byte, 4 * 16> minbuffer = {};
    std::array<std::byte, 4 * 16> maxbuffer = {};
    switch (_componentType) {
    case component_type::the_byte:
      eval_min_max<component_type::the_byte>(*this, result);
      break;
    case component_type::unsigned_byte:
      eval_min_max<component_type::the_byte>(*this, result);
      break;
    case component_type::the_short:
      eval_min_max<component_type::the_short>(*this, result);
      break;
    case component_type::unsigned_short:
      eval_min_max<component_type::unsigned_short>(*this, result);
      break;
    case component_type::unsigned_int:
      eval_min_max<component_type::unsigned_int>(*this, result);
      break;
    case component_type::the_float:
      eval_min_max<component_type::the_float>(*this, result);
      break;
    }
  }

  return result;
}

glTF_json buffer_view::serialize(const document &document_) const {
  auto result = object_base::serialize(document_);
  result["buffer"] = document_.factory().index_of(_allocateResult.buffer);
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
  auto primitivesJson = glTF_json::array();
  for (auto &primitive : _primitives) {
    primitivesJson.push_back(primitive.serialize(document_));
  }
  result["primitives"] = primitivesJson;
  return result;
}

glTF_json primitive::serialize(const document &document_) const {
  glTF_json result;
  glTF_json attributesJson;
  for (auto &attribute : _attributes) {
    attributesJson[to_json_string(attribute.first)] =
        document_.factory().index_of(attribute.second);
  }
  result["attributes"] = attributesJson;
  if (_indices) {
    result["indices"] = document_.factory().index_of(_indices);
  }
  if (_material) {
    result["material"] = document_.factory().index_of(_material);
  }
  if (_mode != mode_type::triangles) {
    result["mode"] = static_cast<integer>(_mode);
  }
  serialize_extras_extensions(result);
  return result;
}

glTF_json skin::serialize(const document &document_) const {
  auto result = object_base::serialize(document_);
  if (!_joints.empty()) {
    glTF_json jointsJson;
    for (auto &joint : _joints) {
      jointsJson.push_back(document_.factory().index_of(joint));
    }
    result["joints"] = jointsJson;
  }
  if (_inverseBindMatrices) {
    result["inverseBindMatrices"] =
        document_.factory().index_of(_inverseBindMatrices);
  }
  return result;
}

glTF_json
material::normal_textre_info::serialize(const document &document_) const {
  auto result = texture_info::serialize(document_);
  if (_scale != _defaultScale) {
    result["scale"] = _scale;
  }
  return result;
}

glTF_json
material::occlusion_textre_info::serialize(const document &document_) const {
  auto result = texture_info::serialize(document_);
  if (_stength != _defaultStrength) {
    result["strength"] = _stength;
  }
  return result;
}

glTF_json material::pbr_metallic_roughness_info::serialize(
    const document &document_) const {
  glTF_json result;
  if (_baseColorFactor != _defaultBaseColorFactor) {
    result["baseColorFactor"] = _baseColorFactor;
  }
  if (_baseColorTexture) {
    result["baseColorTexture"] = _baseColorTexture->serialize(document_);
  }
  if (_metallicFactor != _defaultMetallicFactor) {
    result["metallicFactor"] = _metallicFactor;
  }
  if (_roughnessFactor != _defaultRoughnessFactor) {
    result["roughnessFactor"] = _roughnessFactor;
  }
  if (_metallicRoughnessTexture) {
    result["metallicRoughnessTexture"] =
        _metallicRoughnessTexture->serialize(document_);
  }
  serialize_extras_extensions(result);
  return result;
}

glTF_json material::serialize(const document &document_) const {
  auto result = object_base::serialize(document_);
  if (_pbrMetallicRoughness) {
    result["pbrMetallicRoughness"] =
        _pbrMetallicRoughness->serialize(document_);
  }
  if (_normalTexture) {
    result["normalTexture"] = _normalTexture->serialize(document_);
  }
  if (_occlusionTexture) {
    result["occlusionTexture"] = _occlusionTexture->serialize(document_);
  }
  if (_emissiveTexture) {
    result["emissiveTexture"] = _emissiveTexture->serialize(document_);
  }
  if (_emissiveFactor != _defaultEmissiveFactor) {
    result["emissiveFactor"] = _emissiveFactor;
  }
  if (_alphaMode != _defaultAlphaMode) {
    auto serializeAlphaMode = [this]() {
      switch (_alphaMode) {
      case alpha_mode_type::opaque:
        return "OPAQUE";
      case alpha_mode_type::blend:
        return "BLEND";
      case alpha_mode_type::mask:
        return "MASK";
      default:
        return "";
      }
    };
    result["alphaMode"] = serializeAlphaMode();
  }
  if (_alphaCutoff != _defaultAlphaCutoff) {
    result["alphaCutoff"] = _alphaCutoff;
  }
  if (_doubleSided != _defaultDoubleSided) {
    result["doubleSided"] = _doubleSided;
  }
  return result;
}

glTF_json texture::serialize(const document &document_) const {
  auto result = object_base::serialize(document_);
  if (_source) {
    result["source"] = document_.factory().index_of(_source);
  }
  return result;
}

glTF_json image::serialize(const document &document_) const {
  auto serializeMimeType = [this]() {
    switch (*_mimeType) {
    case allowed_mime_type::jpeg:
      return "image/jpeg";
    case allowed_mime_type::png:
      return "image/png";
    default:
      return "";
    }
  };
  auto result = object_base::serialize(document_);
  std::visit(
      [&](const auto &real_source_) {
        using RealSource = std::decay_t<decltype(real_source_)>;
        if constexpr (std::is_same_v<RealSource, std::u8string>) {
          result["uri"] = to_json_string(real_source_);
        } else if constexpr (std::is_same_v<RealSource,
                                            std::vector<std::byte>>) {
          auto dataBase64 = cppcodec::base64_rfc4648::encode(
              reinterpret_cast<const std::uint8_t *>(real_source_.data()),
              real_source_.size());
          auto dataUri = std::string("data:") + serializeMimeType() +
                         ";base64," + dataBase64;
          result["uri"] = dataUri;
        } else if constexpr (std::is_same_v<RealSource,
                                            object_ptr<buffer_view>>) {
          result["bufferView"] = document_.factory().index_of(real_source_);
        }
      },
      _source);
  if (_mimeType) {
    result["mimeType"] = serializeMimeType();
  }
  return result;
}

glTF_json animation::serialize(const document &document_) const {
  auto result = object_base::serialize(document_);
  auto channelsJson = glTF_json::array();
  for (auto &channelv : factory().get<channel>()) {
    channelsJson.push_back(channelv->serialize(document_, *this));
  }
  result["channels"] = channelsJson;
  auto samplersJson = glTF_json::array();
  for (auto &samplerv : factory().get<sampler>()) {
    samplersJson.push_back(samplerv->serialize(document_));
  }
  result["samplers"] = samplersJson;
  return result;
}

glTF_json animation::channel::serialize(const document &document_,
                                        const animation &animation_) const {
  glTF_json result;
  result["sampler"] = animation_.factory().index_of(_sampler);
  result["target"] = _target.serialize(document_);
  serialize_extras_extensions(result);
  return result;
}

glTF_json
animation::channel::target_type::serialize(const document &document_) const {
  glTF_json result;
  if (_node) {
    result["node"] = document_.factory().index_of(_node);
  }
  auto serializePath = [this]() {
    switch (_path) {
    case builtin_path::translation:
      return "translation";
    case builtin_path::rotation:
      return "rotation";
    case builtin_path::scale:
      return "scale";
    case builtin_path::weights:
      return "weights";
    default:
      return "";
    }
  };
  result["path"] = serializePath();
  serialize_extras_extensions(result);
  return result;
}

glTF_json animation::sampler::serialize(const document &document_) const {
  glTF_json result;
  result["input"] = document_.factory().index_of(_input);
  result["output"] = document_.factory().index_of(_output);
  serialize_extras_extensions(result);
  return result;
}

glTF_json node::serialize(const document &document_) const {
  auto result = object_base::serialize(document_);
  if (_position) {
    if (auto t = *_position; !(t[0] == 0 && t[1] == 0 && t[2] == 0)) {
      result["translation"] = *_position;
    }
  }
  if (_rotation) {
    if (auto r = *_rotation;
        !(r[0] == 0 && r[1] == 0 && r[2] == 0 && r[3] == 1)) {
      result["rotation"] = r;
    }
  }
  if (_scale) {
    if (auto s = *_scale; !(s[0] == 1 && s[1] == 1 && s[2] == 1)) {
      result["scale"] = s;
    }
  }
  if (_matrix) {
    result["matrix"] = *_matrix;
  }
  if (_mesh) {
    result["mesh"] = document_.factory().index_of(_mesh);
  }
  if (_skin) {
    result["skin"] = document_.factory().index_of(_skin);
  }
  if (!_children.empty()) {
    glTF_json childrenJson;
    for (auto &childNode : _children) {
      childrenJson.push_back(document_.factory().index_of(childNode));
    }
    result["children"] = childrenJson;
  }
  return result;
}

glTF_json scene::serialize(const document &document_) const {
  auto result = object_base::serialize(document_);
  if (!_nodes.empty()) {
    glTF_json rootNodesJson;
    for (auto &rootNode : _nodes) {
      rootNodesJson.push_back(document_.factory().index_of(rootNode));
    }
    result["nodes"] = rootNodesJson;
  }
  return result;
}

glTF_json texture_info::serialize(const document &document_) const {
  glTF_json result;
  result["index"] = document_.factory().index_of(_texture);
  if (_texCoord != _defaultTexCoord) {
    result["texCoord"] = _texCoord;
  }
  serialize_extras_extensions(result);
  return result;
}

template <typename Object> struct object_type_t { using type = Object; };

template <typename Object>
inline constexpr static object_type_t<Object> object_type = {};

glTF_json document::serialize(bool rm_uri_of_first_buffer_) const {
  glTF_json result;

  glTF_json assetJson;
  assetJson["generator"] = "3ds-max-plugin-gltf";
  assetJson["version"] = "2.0";
  result["asset"] = assetJson;

  result["scene"] = factory().index_of(_defaultScene);

  auto serializeObjectArray = [&](auto assetType, const char *name_) {
    using AssetType = typename std::decay_t<decltype(assetType)>::type;
    auto &assetArray = factory().get<AssetType>();
    if (assetArray.empty()) {
      return;
    }
    glTF_json arrayJson;
    for (auto i = assetArray.begin(); i != assetArray.end(); ++i) {
      auto &asset = *i;
      if constexpr (std::is_same_v<AssetType, buffer>) {
        auto rmuri = i == assetArray.begin() && rm_uri_of_first_buffer_;
        arrayJson.push_back(asset->serialize(*this, !rmuri));
      } else {
        arrayJson.push_back(asset->serialize(*this));
      }
    }
    result[name_] = arrayJson;
  };

  serializeObjectArray(object_type<accessor>, "accessors");
  serializeObjectArray(object_type<animation>, "animations");
  serializeObjectArray(object_type<buffer>, "buffers");
  serializeObjectArray(object_type<buffer_view>, "bufferViews");
  serializeObjectArray(object_type<camera>, "cameras");
  serializeObjectArray(object_type<image>, "images");
  serializeObjectArray(object_type<material>, "materials");
  serializeObjectArray(object_type<mesh>, "meshes");
  serializeObjectArray(object_type<node>, "nodes");
  serializeObjectArray(object_type<texture_sampler>, "samplers");
  serializeObjectArray(object_type<scene>, "scenes");
  serializeObjectArray(object_type<skin>, "skins");
  serializeObjectArray(object_type<texture>, "textures");

  serialize_extras_extensions(result);
  return result;
}
} // namespace fant::glTF
