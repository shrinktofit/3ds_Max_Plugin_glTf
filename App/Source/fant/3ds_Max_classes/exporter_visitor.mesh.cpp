
#include <CS/BIPEXP.H>
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

glTF::object_ptr<glTF::mesh> exporter_visitor::_convertMesh(
    const _immediate_mesh &imm_mesh_,
    const std::vector<glTF::object_ptr<glTF::material>> &materials_) {
  auto glTFMesh = _document.factory().make<glTF::mesh>();
  glTFMesh->name(imm_mesh_.name);

  for (auto &submesh : imm_mesh_.submeshes) {
    auto &vertices = submesh.vertices;

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

    if (submesh.indices) {
      auto indicesAccessor = _addIndices(gsl::make_span(*submesh.indices));
      primitive.indices(indicesAccessor);
    }

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

std::optional<exporter_visitor::_immediate_mesh> exporter_visitor::_exportMesh(
    IGameNode &igame_node_,
    IGameMesh &igame_mesh_,
    const std::optional<_vertex_skin_data> &skin_data_) {
  // https://knowledge.autodesk.com/support/3ds-max/learn-explore/caas/CloudHelp/cloudhelp/2019/ENU/3DSMax-MAXScript/files/GUID-CBBA20AD-F7D5-46BC-9F5E-5EDA109F9CF4-htm.html
  // https://forums.autodesk.com/t5/3ds-max-programming/weird-igamemesh-problem/td-p/6275934
  // https://help.autodesk.com/view/3DSMAX/2015/ENU/?guid=__cpp_ref_idx__r_list_of_mapping_channel_index_values_html_html
  // > The mesh mapping channel may be specified as one of the following:
  // >   0: Vertex Color channel.
  // >   1: Default mapping channel(the TVert array).
  // >   2 through MAX_MESHMAPS - 1 : The new mapping channels available in
  // release 3.0.

  using PositionChannelComponent = glTF::accessor::component_storage_t<
      glTF::accessor::component_type::the_float>;
  using NormalChannelComponent = PositionChannelComponent;
  using ColorChannelComponent = PositionChannelComponent;
  using TexcoordChannelComponent = PositionChannelComponent;
  using JointChannelComponent = glTF::accessor::component_storage_t<
      glTF::accessor::component_type::unsigned_int>;
  using WeightChannelComponent = NormalChannelComponent;

  if (!igame_mesh_.InitializeData()) {
    // TODO: warn
    return std::nullopt;
  }

  auto name = _convertMaxName(igame_mesh_.GetClassName());

  auto mapNums = igame_mesh_.GetActiveMapChannelNum();
  auto nMapNums = mapNums.Count();

  auto objectOffsetTransform = _getObjectOffsetTransformMatrix(igame_node_, 0);

  struct ChannelList {
    PositionChannelComponent *potition;
    NormalChannelComponent *normal;
    ColorChannelComponent *color; // Optional
    std::unique_ptr<TexcoordChannelComponent *[]> texcoords;
    std::unique_ptr<std::pair<std::byte *, std::byte *>[]> jws;
  };

  auto addChannels = [&](_immediate_mesh::vertex_list &vertices_) {
    auto positionChannel =
        reinterpret_cast<PositionChannelComponent *>(vertices_.add_channel(
            glTF::standard_semantics::position, glTF::accessor::type_type::vec3,
            glTF::accessor::component_type::the_float));
    auto normalChannel =
        reinterpret_cast<NormalChannelComponent *>(vertices_.add_channel(
            glTF::standard_semantics::normal, glTF::accessor::type_type::vec3,
            glTF::accessor::component_type::the_float));
    auto texcoordChannels =
        std::make_unique<TexcoordChannelComponent *[]>(nMapNums);
    ColorChannelComponent *colorChannel = nullptr;
    for (decltype(nMapNums) iMapNum = 0; iMapNum < nMapNums; ++iMapNum) {
      auto iMap = mapNums[iMapNum];
      if (iMap == 0) {
        colorChannel = reinterpret_cast<TexcoordChannelComponent *>(
            vertices_.add_channel(glTF::standard_semantics::color(0),
                                  glTF::accessor::type_type::vec3,
                                  glTF::accessor::component_type::the_float));
      } else {
        texcoordChannels[iMapNum] =
            reinterpret_cast<TexcoordChannelComponent *>(vertices_.add_channel(
                glTF::standard_semantics::texcoord(iMap - 1),
                glTF::accessor::type_type::vec2,
                glTF::accessor::component_type::the_float));
      }
    }
    decltype(ChannelList::jws) jwChannels;
    if (skin_data_) {
      auto nJWSets = skin_data_->sets.size();
      jwChannels =
          std::make_unique<decltype(jwChannels)::element_type[]>(nJWSets);
      for (decltype(nJWSets) iJWSet = 0; iJWSet < nJWSets; ++iJWSet) {
        jwChannels[iJWSet].first = vertices_.add_channel(
            glTF::standard_semantics::joints(iJWSet),
            glTF::accessor::type_type::vec4, skin_data_->joint_storage);
        jwChannels[iJWSet].second = vertices_.add_channel(
            glTF::standard_semantics::weights(iJWSet),
            glTF::accessor::type_type::vec4, skin_data_->weight_storage);
      }
    }
    return ChannelList{positionChannel, normalChannel, colorChannel,
                       std::move(texcoordChannels), std::move(jwChannels)};
  };

  auto addFace = [&](ChannelList &channels_, FaceEx &face_, int face_index_) {
    for (int iFaceCorner = 0; iFaceCorner < 3; ++iFaceCorner) {
      auto outputVertexIndex = 3 * face_index_ + iFaceCorner;

      auto iVertex = face_.vert[iFaceCorner];
      auto vertex = igame_mesh_.GetVertex(iVertex, true);
      auto objectOffsetAppliedVertex =
          _transformPoint(objectOffsetTransform, vertex);
      _convert(objectOffsetAppliedVertex,
               channels_.potition + 3 * outputVertexIndex);

      auto iNormal = face_.norm[iFaceCorner];
      auto normal = igame_mesh_.GetNormal(iNormal, true);
      auto objectOffsetAppliedNormal =
          _transformVector(objectOffsetTransform, vertex).Normalize();
      _convert(objectOffsetAppliedNormal,
               channels_.normal + 3 * outputVertexIndex);

      for (decltype(nMapNums) iMapNum = 0; iMapNum < nMapNums; ++iMapNum) {
        auto iMap = mapNums[iMapNum];

        DWORD mapFaceIndex[3];
        bool success = igame_mesh_.GetMapFaceIndex(iMap, face_.meshFaceIndex,
                                                   mapFaceIndex);
        assert(success);

        Point3 uvw;
        success =
            igame_mesh_.GetMapVertex(iMap, mapFaceIndex[iFaceCorner], uvw);
        assert(success);

        if (iMap == 0) {
          auto pOutputColor = channels_.color + 3 * outputVertexIndex;
          pOutputColor[0] = uvw.x;
          pOutputColor[1] = uvw.y;
          pOutputColor[2] = uvw.z;
        } else {
          auto outputUV = channels_.texcoords[iMapNum] + 2 * outputVertexIndex;
          outputUV[0] = uvw.x;
          outputUV[1] = 1 - uvw.y;
        }
      }

      if (skin_data_) {
        auto &jwsets = skin_data_->sets;
        auto jointBytes =
            glTF::accessor::required_bytes(skin_data_->joint_storage) * 4;
        auto weightBytes =
            glTF::accessor::required_bytes(skin_data_->weight_storage) * 4;
        for (decltype(jwsets.size()) iJWSet = 0; iJWSet < jwsets.size();
             ++iJWSet) {
          auto &jwset = jwsets[iJWSet];
          auto [jointChannel, weightChannel] = channels_.jws[iJWSet];
          std::copy_n(&jwset.joints[jointBytes * iVertex], jointBytes,
                      &jointChannel[jointBytes * outputVertexIndex]);
          std::copy_n(&jwset.weights[weightBytes * iVertex], weightBytes,
                      &weightChannel[weightBytes * outputVertexIndex]);
        }
      }
    }
  };

  std::vector<_immediate_mesh::submesh> submeshes;
  decltype(igame_node_.GetNodeMaterial()->GetSubMaterialCount()) nSubMaterials =
      0;
  if (auto material = igame_node_.GetNodeMaterial()) {
    nSubMaterials = material->GetSubMaterialCount();
  }

  if (nSubMaterials == 0) {
    auto faceCount = igame_mesh_.GetNumberOfFaces();

    _immediate_mesh::vertex_list vertices(3 * faceCount);
    auto channels = addChannels(vertices);

    for (decltype(faceCount) iFace = 0; iFace < faceCount; ++iFace) {
      auto face = igame_mesh_.GetFace(iFace);
      addFace(channels, *face, iFace);
    }

    submeshes.emplace_back(_immediate_mesh::submesh{std::move(vertices)});
  } else { // It's multi-material.
    for (decltype(nSubMaterials) iSubMaterial = 0; iSubMaterial < nSubMaterials;
         ++iSubMaterial) {
      auto materialId = static_cast<MtlID>(iSubMaterial);

      auto materialFaces = igame_mesh_.GetFacesFromMatID(iSubMaterial);
      auto materialFaceCount = materialFaces.Count();

      _immediate_mesh::vertex_list vertices(3 * materialFaceCount);
      auto channels = addChannels(vertices);

      for (decltype(materialFaceCount) iMaterialFacesTab = 0;
           iMaterialFacesTab < materialFaceCount; ++iMaterialFacesTab) {
        auto materialFace = materialFaces[iMaterialFacesTab];
        addFace(channels, *materialFace, iMaterialFacesTab);
      }

      submeshes.emplace_back(
          _immediate_mesh::submesh{materialId, std::move(vertices)});
    }
  }

  return _immediate_mesh{std::move(name), std::move(submeshes)};
}

std::optional<exporter_visitor::_immediate_mesh>
exporter_visitor::_exportMeshIndexed(IGameNode &igame_node_,
                                     IGameMesh &igame_mesh_) {
  if (!igame_mesh_.InitializeData()) {
    // TODO: warn
    return std::nullopt;
  }

  auto name = _convertMaxName(igame_mesh_.GetClassName());

#if true
  return _immediate_mesh{std::move(name), {}};
#else
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
    auto rReduant =
        std::unique(resortedVertices.begin(), resortedVertices.end());
    if (rReduant != resortedVertices.end()) {
      assert(false);
    }
  }
#endif

  return _immediate_mesh{name, std::move(vertices), std::move(submeshes),
                         std::move(vertexResort)};
#endif
}
} // namespace fant