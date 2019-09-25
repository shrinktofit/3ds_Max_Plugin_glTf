
#include <fant/3ds_Max_classes/exporter_visitor.h>

namespace fant {
glTF::object_ptr<glTF::mesh>
exporter_visitor::_convertTriObj(TriObject &tri_obj_) {
  auto &mesh = tri_obj_.GetMesh();
  auto maxNodeName = std::basic_string_view<MCHAR>(
      tri_obj_.NodeName(), tri_obj_.NodeName().Length());
  auto name = win32::mchar_to_utf8(maxNodeName);

  const auto nFaces = mesh.getNumFaces();
  const auto nMaps = mesh.getNumMaps();
  const auto nVData = mesh.getNumVData();

  vertex_list vertices(nFaces * 3, 0);

  for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
       ++iFace) {
    auto &face = mesh.faces[iFace];
    for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
      auto iVertex = face.getVert(iFaceVert);
      auto outVertex = vertices[iFace * 3 + iFaceVert];
      auto &vertex = mesh.getVert(iVertex);
      outVertex.vertex(vertex);
      auto &normal = mesh.getNormal(iVertex);
      outVertex.normal(normal);
    }
  }

  if (mesh.getNumTVerts() != 0) {
    for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
         ++iFace) {
      auto &tface = mesh.tvFace[iFace];
      for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
        auto iTVertex = tface.getTVert(iFaceVert);
        auto outVertex = vertices[iFace * 3 + iFaceVert];
        auto &uvw = mesh.getTVert(iTVertex);
        outVertex.texcoord(uvw);
      }
    }
  }

  for (std::remove_const_t<decltype(nMaps)> iMap = 0; iMap < nMaps; ++iMap) {
  }

  for (std::remove_const_t<decltype(nVData)> iVData = 0; iVData < nVData;
       ++iVData) {
    auto vData = mesh.vData[iVData];
  }

  auto [indexedVertices, indices] = vertices.to_indexed();
  return _addMesh(name, indexedVertices,
                  gsl::make_span(indices.get(), vertices.size()));
}

glTF::object_ptr<glTF::accessor>
exporter_visitor::_addIndices(gsl::span<vertex_list::size_type> indices_) {
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

glTF::object_ptr<glTF::mesh>
exporter_visitor::_addMesh(std::u8string_view name_,
                           const vertex_list &vertex_list_,
                           gsl::span<vertex_list::size_type> indices_) {
  auto indicesAccessor = _addIndices(indices_);

  auto szMainVB = 3 * 4 * vertex_list_.size();
  auto mainVBView =
      _document.factory().make<glTF::buffer_view>(_mainBuffer, szMainVB, 4);

  auto positionAccessor = _document.factory().make<glTF::accessor>(
      mainVBView, 0, glTF::accessor::component_type::the_float,
      glTF::accessor::type_type::vec3, vertex_list_.size(), true);
  for (vertex_list::size_type iVertex = 0; iVertex != vertex_list_.size();
       ++iVertex) {
    auto &v = vertex_list_[iVertex].vertex();
    auto pOutput = reinterpret_cast<float *>(
        mainVBView->data() + static_cast<std::size_t>(3u * 4u * iVertex));
    pOutput[0] = v.x;
    pOutput[1] = v.y;
    pOutput[2] = v.z;
  }

  glTF::primitive primitive{glTF::primitive::mode_type::triangles};
  primitive.indices(indicesAccessor);
  primitive.emplace_attribute(glTF::standard_semantics::position,
                              positionAccessor);

  auto glTFMesh = _document.factory().make<glTF::mesh>();
  glTFMesh->name(name_);
  glTFMesh->push_primitive(primitive);

  return glTFMesh;
}
} // namespace fant