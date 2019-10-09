
#include <CS/BIPEXP.H>
#include <fant/3ds_Max_classes/exporter_visitor.h>

std::vector<Point3> compute_normals(Mesh &mesh_, const Point3 *vetices_) {
  // TODO: smooth group
  // http://docs.autodesk.com/3DSMAX/16/ENU/3ds-Max-SDK-Programmer-Guide/index.html
  // Computing Vertex Normals by Weighting
  std::vector<Point3> result(mesh_.getNumVerts());
  const auto nFaces = mesh_.getNumFaces();
  for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
       ++iFace) {
    auto &face = mesh_.faces[iFace];

    const auto &v0 = vetices_[face.getVert(0)];
    const auto &v1 = vetices_[face.getVert(1)];
    const auto &v2 = vetices_[face.getVert(2)];
    auto faceNormal = Normalize((v1 - v0) ^ (v2 - v1));

    for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
      auto iVertex = face.getVert(iFaceVert);
      result[iVertex] += faceNormal;
    }
  }
  for (auto &normal : result) {
    normal = Normalize(normal);
  }
  return result;
}

std::vector<Point3> compute_normals(IGameMesh &igame_mesh_) {
  // TODO: smooth group
  // http://docs.autodesk.com/3DSMAX/16/ENU/3ds-Max-SDK-Programmer-Guide/index.html
  // Computing Vertex Normals by Weighting
  std::vector<Point3> result(igame_mesh_.GetNumberOfVerts());
  const auto nFaces = igame_mesh_.GetNumberOfFaces();
  for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
       ++iFace) {
    auto &face = *igame_mesh_.GetFace(iFace);
    auto iv0 = face.vert[0];
    auto iv1 = face.vert[1];
    auto iv2 = face.vert[2];
    const auto &v0 = igame_mesh_.GetVertex(iv0, true);
    const auto &v1 = igame_mesh_.GetVertex(iv1, true);
    const auto &v2 = igame_mesh_.GetVertex(iv2, true);
    auto faceNormal = Normalize((v1 - v0) ^ (v2 - v1));
    result[iv0] += faceNormal;
    result[iv1] += faceNormal;
    result[iv2] += faceNormal;
  }
  for (auto &normal : result) {
    normal = Normalize(normal);
  }
  return result;
}

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

  // We don't want the skeleton.
  // TODO: filter BIPED_CLASS_ID?
  if (object->ClassID() == SKELOBJ_CLASS_ID) {
    return std::nullopt;
  }

  auto triObject =
      static_cast<TriObject *>(object->ConvertToType(0, {TRIOBJ_CLASS_ID, 0}));
  if (!triObject) {
    return std::nullopt;
  }

  auto immMesh = _convertTriObj(max_node_, *triObject);
  if (triObject != object) {
    triObject->DeleteMe();
  }

  return immMesh;
}

exporter_visitor::_immediate_mesh
exporter_visitor::_convertTriObj(INode &max_node_, TriObject &tri_obj_) {
  // https://knowledge.autodesk.com/support/3ds-max/learn-explore/caas/CloudHelp/cloudhelp/2019/ENU/3DSMax-MAXScript/files/GUID-CBBA20AD-F7D5-46BC-9F5E-5EDA109F9CF4-htm.html
  auto &mesh = tri_obj_.GetMesh();
  auto maxNodeName = tri_obj_.NodeName();
  auto name = _convertMaxName(maxNodeName);

  const auto nVerts = mesh.getNumVerts();

  _immediate_mesh::vertex_list vertices(nVerts);

  std::vector<Point3> positions(nVerts); // We need it to calculate normal
  for (std::remove_const_t<decltype(nVerts)> iVertex = 0; iVertex < nVerts;
       ++iVertex) {
    positions[iVertex] = _convertIntoGlTFAxisSystem(mesh.getVert(iVertex));
  }

  auto normals = compute_normals(mesh, positions.data());

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
    _convert(positions[iVertex], positionChannel + 3 * iVertex);
    _convert(normals[iVertex], normalChannel + 3 * iVertex);
  }

  auto mtl = max_node_.GetMtl();

  const auto nFaces = mesh.getNumFaces();
  std::unordered_map<MtlID, _immediate_mesh::submesh> submeshes;
  /*std::vector<_immediate_mesh::vertex_list::vertex_count_type> indices(3 *
                                                                       nFaces);*/
  for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
       ++iFace) {
    auto mtlIndex = mesh.getFaceMtlIndex(iFace);
    auto submesh = submeshes.try_emplace(mtlIndex);
    if (submesh.second) {
      submesh.first->second.material_id = mtlIndex;
      submesh.first->second.indices.reserve(3 * nFaces);
    }
    auto &indices = submesh.first->second.indices;

    auto &face = mesh.faces[iFace];
    for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
      auto iVertex = face.getVert(iFaceVert);
      indices.push_back(iVertex);
      // indices[3 * iFace + iFaceVert] = iVertex;
    }
  }

  const auto nMaps = mesh.getNumMaps();
  // https://help.autodesk.com/view/3DSMAX/2015/ENU/?guid=__cpp_ref_idx__r_list_of_mapping_channel_index_values_html_html
  // > The mesh mapping channel may be specified as one of the following:
  // >   0: Vertex Color channel.
  // >   1: Default mapping channel(the TVert array).
  // >   2 through MAX_MESHMAPS - 1 : The new mapping channels available in
  // release 3.0.
  for (std::remove_const_t<decltype(nMaps)> iMap = 1; iMap < nMaps; ++iMap) {
    auto mapFaces = mesh.mapFaces(iMap);
    if (!mapFaces) {
      continue;
    }
    auto set = iMap - 1;
    auto texcoordChannel = reinterpret_cast<glTF::accessor::component_storage_t<
        glTF::accessor::component_type::the_float> *>(
        vertices.add_channel(glTF::standard_semantics::texcoord(set),
                             glTF::accessor::type_type::vec2,
                             glTF::accessor::component_type::the_float));
    for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
         ++iFace) {
      auto &mapFace = mapFaces[iFace];
      auto &face = mesh.faces[iFace];
      auto faceMtl = !mtl ? nullptr
                          : (mtl->NumSubMtls() == 0
                                 ? mtl
                                 : mtl->GetSubMtl(mesh.getFaceMtlIndex(iFace)));
      for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
        auto iTVertex = mapFace.getTVert(iFaceVert);
        auto iVertex = face.getVert(iFaceVert);
        auto uvw = mesh.getTVert(iTVertex);

        /*if (faceMtl) {
          auto nSubTexmaps = faceMtl->NumSubTexmaps();
          for (decltype(nSubTexmaps) iSubTexmap = 0; iSubTexmap < nSubTexmaps;
               ++iSubTexmap) {
            auto texmap = faceMtl->GetSubTexmap(iSubTexmap);
            Matrix3 uvmatrix(TRUE);
            texmap->GetUVTransform(uvmatrix);
            auto uvgen = texmap->GetTheUVGen();

          }
        }*/

        auto pOut = texcoordChannel + 2 * iVertex;
        pOut[0] = uvw.x;
        pOut[1] = 1.0 - uvw.y;
      }
    }
  }

  if (mesh.getNumTVerts() != 0) {
    // uv0 channel
    /*auto texcoordChannel =
    reinterpret_cast<glTF::accessor::component_storage_t<
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

  std::vector<_immediate_mesh::submesh> submeshesArray(submeshes.size());
  std::transform(submeshes.begin(), submeshes.end(), submeshesArray.begin(),
                 [](auto &kv) { return std::move(kv.second); });

  return {name, std::move(vertices), std::move(submeshesArray)};
}

glTF::object_ptr<glTF::mesh> exporter_visitor::_convertMesh(
    const _immediate_mesh &imm_mesh_,
    const std::vector<glTF::object_ptr<glTF::material>> &materials_) {
  auto &vertices = imm_mesh_.vertices;

  std::vector<
      std::pair<const std::u8string_view, glTF::object_ptr<glTF::accessor>>>
      attributes;
  attributes.reserve(vertices.channels_size());
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
    attributes.emplace_back(iChannel->first, accessor);
  }

  auto glTFMesh = _document.factory().make<glTF::mesh>();
  glTFMesh->name(imm_mesh_.name);

  for (auto &submesh : imm_mesh_.submeshes) {
    glTF::primitive primitive{glTF::primitive::mode_type::triangles};
    for (auto &attribute : attributes) {
      primitive.emplace_attribute(attribute.first, attribute.second);
    }

    auto indicesAccessor = _addIndices(gsl::make_span(submesh.indices));
    primitive.indices(indicesAccessor);

    if (submesh.material_id < materials_.size()) {
      primitive.material(materials_[submesh.material_id]);
    } else {
      // TODO warn:
    }

    glTFMesh->push_primitive(primitive);
  }

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

  _immediate_mesh::vertex_list::vertex_count_type maxIndex = 0;
  for (auto index : indices_) {
    if (index > maxIndex) {
      maxIndex = index;
    }
  }

  static const std::array<decltype(maxIndex), 3> capacitys = {
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
    if (maxIndex <= capacitys[getCapacityIndex(_settings.index_type)]) {
      usingType = _settings.index_type;
    }
  } else {
    for (auto iCap = getCapacityIndex(_settings.index_type);
         iCap < capacitys.size(); ++iCap) {
      auto cap = capacitys[iCap];
      if (cap >= maxIndex) {
        usingType = getTypeFromCapacityIndex(iCap);
        break;
      }
    }
  }

  if (!usingType) {
    throw std::runtime_error("Index too large.");
  }

  assert(maxIndex <= capacitys[getCapacityIndex(*usingType)]);

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

std::optional<exporter_visitor::_immediate_mesh>
exporter_visitor::_exportMesh(IGameNode &igame_node_, IGameMesh &igame_mesh_) {
  // https://knowledge.autodesk.com/support/3ds-max/learn-explore/caas/CloudHelp/cloudhelp/2019/ENU/3DSMax-MAXScript/files/GUID-CBBA20AD-F7D5-46BC-9F5E-5EDA109F9CF4-htm.html
  // https://forums.autodesk.com/t5/3ds-max-programming/weird-igamemesh-problem/td-p/6275934
  // https://help.autodesk.com/view/3DSMAX/2015/ENU/?guid=__cpp_ref_idx__r_list_of_mapping_channel_index_values_html_html
  // > The mesh mapping channel may be specified as one of the following:
  // >   0: Vertex Color channel.
  // >   1: Default mapping channel(the TVert array).
  // >   2 through MAX_MESHMAPS - 1 : The new mapping channels available in
  // release 3.0.

  if (!igame_mesh_.InitializeData()) {
    // TODO: warn
    return std::nullopt;
  }

  auto nFaces = igame_mesh_.GetNumberOfFaces();
  auto mapNums = igame_mesh_.GetActiveMapChannelNum();
  auto nVertexAttribs = mapNums.Count() + 1;

  struct PlanarVertices {
  public:
    using index_type = std::vector<int>::size_type;

    PlanarVertices(index_type attribute_count_, index_type face_count_hint_)
        : _nAttribs(attribute_count_) {
      _data.reserve(_nAttribs * face_count_hint_ * 3);
    }

    index_type try_emplace(const int *source_) {
      for (index_type iPlanarVertex = 0; iPlanarVertex < _size;
           ++iPlanarVertex) {
        if (std::memcmp(_data.data() + _nAttribs * iPlanarVertex, source_,
                        _nAttribs) == 0) {
          return iPlanarVertex;
        }
      }
      _data.insert(_data.end(), source_, source_ + _nAttribs);
      auto result = _size;
      ++_size;
      return result;
    }

    index_type size() const {
      return _size;
    }

    int get(index_type index_, index_type attribute_index_) const {
      return (_data.data() + _nAttribs * index_)[attribute_index_];
    }

  private:
    std::vector<int> _data;
    index_type _nAttribs;
    index_type _size = 0;
  };

  PlanarVertices planarVertices(nVertexAttribs, nFaces);
  auto buffPlannarVertex = std::make_unique<int[]>(nVertexAttribs);
  std::vector<PlanarVertices::index_type> plannarIndices(3 * nFaces);
  for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
       ++iFace) {
    for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
      auto iVertex = igame_mesh_.GetFaceVertex(iFace, iFaceVert);
      buffPlannarVertex[0] = iVertex;
      for (decltype(mapNums.Count()) iMapNum = 0; iMapNum < mapNums.Count();
           ++iMapNum) {
        auto iMap = mapNums[iMapNum];
        auto iTVertex =
            igame_mesh_.GetFaceTextureVertex(iFace, iFaceVert, iMap);
        buffPlannarVertex[iMapNum + 1] = iTVertex;
      }
      auto iPlannarVertex = planarVertices.try_emplace(buffPlannarVertex.get());
      plannarIndices[3 * iFace + iFaceVert] = iPlannarVertex;
    }
  }

  auto nPlanarVertices = planarVertices.size();
  _immediate_mesh::vertex_list vertices(nPlanarVertices);

  auto normals = compute_normals(igame_mesh_);

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
  for (std::remove_const_t<decltype(nPlanarVertices)> iPlannarVertex = 0;
       iPlannarVertex < nPlanarVertices; ++iPlannarVertex) {
    auto iVertex = planarVertices.get(iPlannarVertex, 0);

    auto vertex = igame_mesh_.GetVertex(iVertex, true);
    _convert(vertex, positionChannel + 3 * iPlannarVertex);
    _convert(normals[iVertex], normalChannel + 3 * iPlannarVertex);
  }

  for (decltype(mapNums.Count()) iMapNum = 0; iMapNum < mapNums.Count();
       ++iMapNum) {
    auto iMap = mapNums[iMapNum];
    auto set = iMap - 1;
    auto texcoordChannel = reinterpret_cast<glTF::accessor::component_storage_t<
        glTF::accessor::component_type::the_float> *>(
        vertices.add_channel(glTF::standard_semantics::texcoord(set),
                             glTF::accessor::type_type::vec2,
                             glTF::accessor::component_type::the_float));
    for (std::remove_const_t<decltype(nPlanarVertices)> iPlannarVertex = 0;
         iPlannarVertex < nPlanarVertices; ++iPlannarVertex) {
      auto iTVertex = planarVertices.get(iPlannarVertex, 1 + iMapNum);

      auto mapVertex = igame_mesh_.GetMapVertex(iMap, iTVertex);
      auto pOut = texcoordChannel + 2 * iPlannarVertex;
      pOut[0] = mapVertex.x;
      pOut[1] = 1.0 - mapVertex.y;
    }
  }

  decltype(igame_node_.GetNodeMaterial()->GetSubMaterialCount()) nSubMaterials =
      0;
  if (auto material = igame_node_.GetNodeMaterial()) {
    nSubMaterials = material->GetSubMaterialCount();
  }

  decltype(_immediate_mesh::submeshes) submeshes;
  if (nSubMaterials == 0) {
    // https://forums.autodesk.com/t5/3ds-max-programming/igamemesh-getfacesfrommatid-returns-different-from-igamemesh/td-p/9058585
    auto nFaces = igame_mesh_.GetNumberOfFaces();
    _immediate_mesh::submesh submesh;
    submesh.material_id = 0;
    submesh.indices.resize(3 * nFaces);
    for (decltype(nFaces) iFace = 0; iFace < nFaces; ++iFace) {
      for (int iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
        submesh.indices[3 * iFace + iFaceVert] =
            plannarIndices[3 * iFace + iFaceVert];
      }
    }
    submeshes.emplace_back(std::move(submesh));
  } else {
    std::unordered_map<decltype(igame_mesh_.GetFaceMaterialID(0)),
                       _immediate_mesh::submesh>
        submeshMap;
    for (decltype(nFaces) iFace = 0; iFace < nFaces; ++iFace) {
      auto materialId = igame_mesh_.GetFaceMaterialID(iFace);
      auto &submesh = submeshMap[materialId];
      if (submesh.indices.empty()) {
        submesh.material_id = materialId;
        submesh.indices.reserve(
            3 * igame_mesh_.GetFacesFromMatID(materialId).Count());
      }
      for (int iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
        submesh.indices.push_back(plannarIndices[3 * iFace + iFaceVert]);
      }
    }
    submeshes.resize(submeshMap.size());
    std::transform(submeshMap.begin(), submeshMap.end(), submeshes.begin(),
                   [](auto &kv_) { return std::move(kv_.second); });
  }

  /*
  decltype(igame_node_.GetNodeMaterial()->GetSubMaterialCount()) nSubMaterials =
      0;
  if (auto material = igame_node_.GetNodeMaterial()) {
    nSubMaterials = material->GetSubMaterialCount();
  }

  decltype(_immediate_mesh::submeshes) submeshes;
  if (nSubMaterials == 0) {
    //
  https://forums.autodesk.com/t5/3ds-max-programming/igamemesh-getfacesfrommatid-returns-different-from-igamemesh/td-p/9058585
    auto nFaces = igame_mesh_.GetNumberOfFaces();
    _immediate_mesh::submesh submesh;
    submesh.material_id = 0;
    submesh.indices.resize(3 * nFaces);
    for (decltype(nFaces) iFace = 0; iFace < nFaces; ++iFace) {
      auto face = igame_mesh_.GetFace(iFace);
      for (int iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
        submesh.indices[3 * iFace + iFaceVert] = face->vert[iFaceVert];
      }
    }
    submeshes.emplace_back(std::move(submesh));
  } else {
    submeshes.resize(nSubMaterials);
    for (decltype(nSubMaterials) iMaterial = 0; iMaterial < nSubMaterials;
         ++iMaterial) {
      _immediate_mesh::submesh submesh;
      submesh.material_id = iMaterial;
      auto faces = igame_mesh_.GetFacesFromMatID(iMaterial);
      auto nFaces = faces.Count();
      submesh.indices.resize(3 * nFaces);
      for (decltype(nFaces) iFace = 0; iFace < nFaces; ++iFace) {
        auto face = faces[iFace];
        for (int iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
          submesh.indices[3 * iFace + iFaceVert] = face->vert[iFaceVert];
        }
      }
      submeshes[iMaterial] = std::move(submesh);
    }
  }

  const auto nVerts = igame_mesh_.GetNumberOfVerts();

  _immediate_mesh::vertex_list vertices(nVerts);

  auto normals = compute_normals(igame_mesh_);

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
    auto vertex = igame_mesh_.GetVertex(iVertex);
    _convert(vertex, positionChannel + 3 * iVertex);
    _convert(normals[iVertex], normalChannel + 3 * iVertex);
  }

  for (decltype(mapNums.Count()) iMapNum = 0; iMapNum < mapNums.Count();
       ++iMapNum) {
    auto iMap = mapNums[iMapNum];
    auto set = iMap - 1;
    auto texcoordChannel = reinterpret_cast<glTF::accessor::component_storage_t<
        glTF::accessor::component_type::the_float> *>(
        vertices.add_channel(glTF::standard_semantics::texcoord(set),
                             glTF::accessor::type_type::vec2,
                             glTF::accessor::component_type::the_float));
    auto nFaces = igame_mesh_.GetNumberOfFaces();
    for (std::remove_const_t<decltype(nFaces)> iFace = 0; iFace < nFaces;
         ++iFace) {
      for (auto iFaceVert = 0; iFaceVert < 3; ++iFaceVert) {
        auto iTVertex =
            igame_mesh_.GetFaceTextureVertex(iFace, iFaceVert, iMap);
        auto tVertex = igame_mesh_.GetTexVertex(iTVertex);
        auto iVertex = igame_mesh_.GetFaceVertex(iFace, iFaceVert);

        auto pOut = texcoordChannel + 2 * iVertex;
        pOut[0] = tVertex.x;
        pOut[1] = 1.0 - tVertex.y;
      }
    }
  }*/

  decltype(_immediate_mesh::vertex_resort) vertexResort(nPlanarVertices);
  for (decltype(nPlanarVertices) iPlanarVertex = 0;
       iPlanarVertex < nPlanarVertices; ++iPlanarVertex) {
    vertexResort[iPlanarVertex] = planarVertices.get(iPlanarVertex, 0);
  }

#ifndef NDEBUG
  {
    auto resortedVertices = vertexResort;
    std::sort(resortedVertices.begin(), resortedVertices.end());
    auto rReduant = std::unique(resortedVertices.begin(), resortedVertices.end());
    if (rReduant != resortedVertices.end()) {
      assert(false);
    }
  }
#endif

  auto name = _convertMaxName(igame_mesh_.GetClassName());
  return _immediate_mesh{name, std::move(vertices), std::move(submeshes),
                         std::move(vertexResort)};
}
} // namespace fant