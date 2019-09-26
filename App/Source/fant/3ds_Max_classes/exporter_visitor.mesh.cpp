
#include <fant/3ds_Max_classes/exporter_visitor.h>

namespace fant {

std::pair<
    exporter_visitor::_immediate_mesh::vertex_list,
    std::vector<
        exporter_visitor::_immediate_mesh::vertex_list::vertex_count_type>>
exporter_visitor::_immediate_mesh::vertex_list::to_indexed() const {
  auto positionChannel = _channels.find(glTF::standard_semantics::position);
  if (positionChannel == _channels.end()) {
    throw std::runtime_error("to_indexed() needs position attribute.");
  }

  auto positionChannelData = positionChannel->second.data.get();
  auto positionBytes = positionChannel->second.attribute_bytes();

  std::vector<std::byte *> uniqueVertices;
  uniqueVertices.reserve(_nVertices);
  std::vector<vertex_count_type> indices(_nVertices);
  for (vertex_count_type iVertex = 0; iVertex < _nVertices; ++iVertex) {
    auto pVertex = positionChannelData + positionBytes * iVertex;
    auto rUnique =
        std::find_if(uniqueVertices.begin(), uniqueVertices.end(),
                     [pVertex, positionBytes](auto unique_) {
                       return std::memcmp(unique_, pVertex, positionBytes) == 0;
                     });
    if (rUnique != uniqueVertices.end()) {
      indices[iVertex] = (rUnique - uniqueVertices.begin());
    } else {
      indices[iVertex] = (uniqueVertices.size());
      uniqueVertices.push_back(pVertex);
    }
  }

  vertex_list indexedVertices(uniqueVertices.size());
  for (auto &channel : _channels) {
    auto attributeBytes = channel.second.attribute_bytes();
    auto originalChannelData = channel.second.data.get();
    auto channelData = indexedVertices.add_channel(
        channel.first, channel.second.type, channel.second.component);
    for (vertex_count_type iUniqueVertex = 0;
         iUniqueVertex < uniqueVertices.size(); ++iUniqueVertex) {
      auto iOriginalVertex =
          (uniqueVertices[iUniqueVertex] - positionChannelData) / positionBytes;
      std::copy_n(originalChannelData + attributeBytes * iOriginalVertex,
                  attributeBytes, channelData + attributeBytes * iUniqueVertex);
    }
  }

  return {
      std::move(indexedVertices),
      std::move(indices),
  };
}

std::optional<exporter_visitor::_immediate_mesh>
exporter_visitor::_tryExportMesh(INode &max_node_) {
  auto object = max_node_.EvalWorldState(0).obj;
  if (!object->CanConvertToType({TRIOBJ_CLASS_ID, 0})) {
    return std::nullopt;
  }

  auto triObject =
      static_cast<TriObject *>(object->ConvertToType(0, {TRIOBJ_CLASS_ID, 0}));
  if (!triObject) {
    return std::nullopt;
  }

  auto immMesh = _convertTriObj(*triObject);
  if (triObject != object) {
    triObject->DeleteMe();
  }

  return immMesh;
}

exporter_visitor::_immediate_mesh
exporter_visitor::_convertTriObj(TriObject &tri_obj_) {
  auto &mesh = tri_obj_.GetMesh();
  auto maxNodeName = std::basic_string_view<MCHAR>(
      tri_obj_.NodeName(), tri_obj_.NodeName().Length());
  auto name = win32::mchar_to_utf8(maxNodeName);

  const auto nVerts = mesh.getNumVerts();
  const auto nMaps = mesh.getNumMaps();
  const auto nVData = mesh.getNumVData();

  _immediate_mesh::vertex_list vertices(nVerts);

  auto positionChannel = reinterpret_cast<glTF::accessor::component_storage_t<
      glTF::accessor::component_type::the_float> *>(
      vertices.add_channel(glTF::standard_semantics::position,
                           glTF::accessor::type_type::vec3,
                           glTF::accessor::component_type::the_float));

  auto normalChannel = reinterpret_cast<glTF::accessor::component_storage_t<
      glTF::accessor::component_type::the_float> *>(
      vertices.add_channel(glTF::standard_semantics::normal,
                           glTF::accessor::type_type::vec3,
                           glTF::accessor::component_type::the_float));

  for (std::remove_const_t<decltype(nVerts)> iVertex = 0; iVertex < nVerts;
       ++iVertex) {
    auto &vertex = mesh.getVert(iVertex);
    _convert(vertex, positionChannel + 3 * iVertex);
    auto &normal = Normalize(mesh.getNormal(iVertex));
    _convert(normal, normalChannel + 3 * iVertex);
  }

  const auto nFaces = mesh.getNumFaces();
  std::vector<_immediate_mesh::vertex_list::vertex_count_type> indices(3 * nFaces);
  for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
       ++iFace) {
    auto &face = mesh.faces[iFace];
    for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
      auto iVertex = face.getVert(iFaceVert);
      indices[3 * iFace + iFaceVert] = iVertex;
    }
  }

  if (mesh.getNumTVerts() != 0) {
    // uv0 channel
    /*auto texcoordChannel = reinterpret_cast<glTF::accessor::component_storage_t<
        glTF::accessor::component_type::the_float> *>(
        vertices.add_channel(glTF::standard_semantics::texcoord(0),
                             glTF::accessor::type_type::vec2,
                             glTF::accessor::component_type::the_float));
    for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
         ++iFace) {
      auto &tface = mesh.tvFace[iFace];
      for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
        auto iTVertex = tface.getTVert(iFaceVert);
        auto &uvw = mesh.getTVert(iTVertex);
        auto iOutputVertex = iFace * 3 + iFaceVert;
        auto pOut = texcoordChannel + 2 * iOutputVertex;
        pOut[0] = uvw.x;
        pOut[1] = uvw.y;
      }
    }*/
  }

  for (std::remove_const_t<decltype(nMaps)> iMap = 0; iMap < nMaps; ++iMap) {
  }

  for (std::remove_const_t<decltype(nVData)> iVData = 0; iVData < nVData;
       ++iVData) {
    auto vData = mesh.vData[iVData];
  }

  return {name, std::move(vertices), std::move(indices)};
}

glTF::object_ptr<glTF::mesh>
exporter_visitor::_convertMesh(const _immediate_mesh &imm_mesh_) {
  auto &vertices = imm_mesh_.vertices;

  glTF::primitive primitive{glTF::primitive::mode_type::triangles};

  for (auto iChannel = vertices.channels_begin();
       iChannel != vertices.channels_end(); ++iChannel) {
    auto accessor =
        _makeSimpleAccessor(iChannel->second.type, iChannel->second.component,
                            vertices.vertex_count());
    std::copy_n(iChannel->second.data.get(),
                iChannel->second.attribute_bytes() * vertices.vertex_count(),
                accessor->data());
    accessor->name(iChannel->first);
    if (iChannel->first == glTF::standard_semantics::position) {
      accessor->explicit_bound_required(true);
    }
    primitive.emplace_attribute(iChannel->first, accessor);
  }

  if (imm_mesh_.indices) {
    auto indicesAccessor = _addIndices(gsl::make_span(*imm_mesh_.indices));
    primitive.indices(indicesAccessor);
  }

  auto glTFMesh = _document.factory().make<glTF::mesh>();
  glTFMesh->name(imm_mesh_.name);
  glTFMesh->push_primitive(primitive);

  return glTFMesh;
}

glTF::object_ptr<glTF::accessor> exporter_visitor::_addIndices(
    gsl::span<const _immediate_mesh::vertex_list::vertex_count_type> indices_) {
  static_assert(export_settings::index_type_setting::unsigned_8 ==
                static_cast<export_settings::index_type_setting>(0));
  static_assert(export_settings::index_type_setting::unsigned_16 ==
                static_cast<export_settings::index_type_setting>(1));
  static_assert(export_settings::index_type_setting::unsigned_32 ==
                static_cast<export_settings::index_type_setting>(2));

  static const std::array<decltype(indices_.size()), 3> capacitys = {
      std::numeric_limits<std::uint8_t>::max(),
      std::numeric_limits<std::uint16_t>::max(),
      std::numeric_limits<std::uint32_t>::max()};

  static auto getCapacityIndex = [](export_settings::index_type_setting type_) {
    return static_cast<
        std::underlying_type_t<export_settings::index_type_setting>>(type_);
  };

  static auto getTypeFromCapacityIndex = [](decltype(capacitys.size()) index_) {
    return static_cast<export_settings::index_type_setting>(index_);
  };

  std::optional<export_settings::index_type_setting> usingType;
  if (_settings.index == export_settings::index_setting::fixed) {
    if (indices_.size() <= capacitys[getCapacityIndex(_settings.index_type)]) {
      usingType = _settings.index_type;
    }
  } else {
    for (auto iCap = getCapacityIndex(_settings.index_type);
         iCap < capacitys.size(); ++iCap) {
      auto cap = capacitys[iCap];
      if (cap >= indices_.size()) {
        usingType = getTypeFromCapacityIndex(iCap);
        break;
      }
    }
  }

  if (!usingType) {
    throw std::runtime_error("Index too large.");
  }

  assert(indices_.size() <= capacitys[getCapacityIndex(*usingType)]);

  std::uint32_t sizeofIndex = 0;
  switch (*usingType) {
  case export_settings::index_type_setting::unsigned_8: {
    sizeofIndex = 1;
    break;
  }
  case export_settings::index_type_setting::unsigned_16: {
    sizeofIndex = 2;
    break;
  }
  case export_settings::index_type_setting::unsigned_32: {
    sizeofIndex = 4;
    break;
  }
  }

  auto bufferView = _document.factory().make<glTF::buffer_view>(
      _mainBuffer, static_cast<std::uint32_t>(sizeofIndex * indices_.size()),
      sizeofIndex);
  bufferView->target(glTF::buffer_view::target_type::element_array_buffer);

  auto glTfComponentType = glTF::accessor::component_type::unsigned_byte;
  switch (*usingType) {
  case export_settings::index_type_setting::unsigned_8: {
    glTfComponentType = glTF::accessor::component_type::unsigned_byte;
    std::copy_n(indices_.data(), indices_.size(),
                reinterpret_cast<std::uint8_t *>(bufferView->data()));
    break;
  }
  case export_settings::index_type_setting::unsigned_16: {
    glTfComponentType = glTF::accessor::component_type::unsigned_short;
    std::copy_n(indices_.data(), indices_.size(),
                reinterpret_cast<std::uint16_t *>(bufferView->data()));
    break;
  }
  case export_settings::index_type_setting::unsigned_32: {
    glTfComponentType = glTF::accessor::component_type::unsigned_int;
    std::copy_n(indices_.data(), indices_.size(),
                reinterpret_cast<std::uint32_t *>(bufferView->data()));
    break;
  }
  }

  auto accessor = _document.factory().make<glTF::accessor>(
      bufferView, 0, glTfComponentType, glTF::accessor::type_type::scalar,
      static_cast<std::uint32_t>(indices_.size()));
  return accessor;
}
} // namespace fant