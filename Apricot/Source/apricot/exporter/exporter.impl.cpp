
#include <IGame/IGame.h>
#include <apricot/exporter/exporter.impl.h>
#include <apricot/utilities/max/mchar-to-u8.h>
#include <cppcodec/base64_rfc4648.hpp>
#include <filesystem>
#include <functional>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include <trig.h>

namespace apricot {
exporter_impl::exporter_impl(Interface &max_interface_,
                             glTF::document &document_,
                             const export_settings &settings_,
                             IScene &scene_)
    : _maxInterface(max_interface_), _document(document_),
      _glTFScene(_document.factory().make<glTF::scene>()),
      _settings(settings_) {
  _document.default_scene(_glTFScene);
  _mainBuffer = _document.factory().make<glTF::buffer>();
  _mainAnimation = _document.factory().make<glTF::animation>();

  auto animRange = _maxInterface.GetAnimRange();
  auto fps = 30;
  if (animRange.Duration()) {
    auto step = SecToTicks((1 / float(30)));
    auto nFrame = static_cast<glTF::accessor::size_type>(
        std::ceil(animRange.Duration() / step));
    auto inputAccessor =
        _makeSimpleAccessor(glTF::accessor::type_type::scalar,
                            glTF::accessor::component_type::the_float, nFrame);
    inputAccessor->explicit_bound_required(true);
    auto inputData =
        inputAccessor->typed_data<glTF::accessor::component_type::the_float>();
    for (decltype(nFrame) iFrame = 0; iFrame < nFrame; ++iFrame) {
      inputData[iFrame] = TicksToSec(step * iFrame);
    }
    _animBaking.emplace();
    _animBaking->start = animRange.Start();
    _animBaking->step = step;
    _animBaking->frame_count = nFrame;
    _animBaking->input = inputAccessor;
  }

  auto igameScene = GetIGameInterface();

  UserCoord glTFCoord;
  glTFCoord.rotation = 1; // right handed
  glTFCoord.xAxis = 1;    // right
  glTFCoord.yAxis = 2;    // up
  glTFCoord.zAxis = 5;    // out
  glTFCoord.uAxis = 0;    // left
  glTFCoord.vAxis = 0;    // up

  auto conversionManager = GetConversionManager();
  // conversionManager->SetUserCoordSystem(glTFCoord);
  conversionManager->SetCoordSystem(
      IGameConversionManager::CoordSystem::IGAME_OGL);

  if (!igameScene->InitialiseIGame()) {
    throw std::runtime_error("Failed to initialise 3DXI.");
  }

  igameScene->SetStaticFrame(0);

  for (decltype(igameScene->GetTopLevelNodeCount()) iTopLevelNode = 0;
       iTopLevelNode < igameScene->GetTopLevelNodeCount(); ++iTopLevelNode) {
    auto igameNode = igameScene->GetTopLevelNode(iTopLevelNode);
    auto glTFRootNode = _exportNode(*igameNode);
    _glTFScene->add_node(glTFRootNode);
  }

  /*auto nRootMaterials = igameScene->GetRootMaterialCount();
  std::vector<std::vector<glTF::object_ptr<glTF::material>>> glTFRootMaterials(
      nRootMaterials);
  for (decltype(nRootMaterials) iRootMaterial = 0;
       iRootMaterial < nRootMaterials; ++iRootMaterial) {
    auto rootMaterial = igameScene->GetRootMaterial(iRootMaterial);
    glTFRootMaterials[iRootMaterial] = _exportMaterial(*rootMaterial);
  }*/

  for (auto &nodeMap : _nodeMaps2) {
    auto &igameNode = *nodeMap.first;
    auto &glTFNode = nodeMap.second;
    auto object = igameNode.GetIGameObject();
    switch (object->GetIGameType()) {
    case IGameObject::ObjectTypes::IGAME_MESH: {
      auto &igameMesh = static_cast<IGameMesh &>(*object);
      glTF::object_ptr<glTF::skin> glTFSkin;
      std::optional<_vertex_skin_data> vertexSkinData;
      if (igameMesh.IsObjectSkinned()) {
        std::tie(glTFSkin, vertexSkinData) =
            _exportSkin(igameNode, *igameMesh.GetIGameSkin(), glTFNode);
      }
      if (auto immMesh = _exportMesh(igameNode, igameMesh, vertexSkinData)) {
        auto glTFMaterials = _tryExportMaterial(*igameNode.GetMaxNode());
        auto glTFMesh = _convertMesh(*immMesh, glTFMaterials);
        glTFNode->mesh(glTFMesh);
        if (glTFSkin) {
          glTFNode->skin(glTFSkin);
        }
      }
    } break;
    default:
      break;
    }
    _bakeAnimation(igameNode, _animBaking->start, _animBaking->step,
                   _animBaking->frame_count, glTFNode, _animBaking->input);
  }

  igameScene->ReleaseIGame();

  if (_document.factory().get_size<glTF::buffer>() == 0) {
    auto demandBuffer = _document.factory().make<glTF::buffer>();
    demandBuffer->allocate(1, 0);
  }
}

static const Matrix3 &get_transform_to_max_axis_system() {
  static auto toMaxAxisSystem = []() {
    Matrix3 matrix(TRUE);
    matrix.SetRotateX(DegToRad(90));
    return matrix;
  }();
  return toMaxAxisSystem;
}

static const Matrix3 &get_transform_to_max_axis_system_inverse() {
  static auto result = Inverse(get_transform_to_max_axis_system());
  return result;
}

GMatrix apricot::exporter_impl::_convertIntoGlTFAxisSystem(
    const GMatrix &max_transform_) {
  return GMatrix(get_transform_to_max_axis_system()) * max_transform_ *
         GMatrix(get_transform_to_max_axis_system_inverse());
}

Matrix3
exporter_impl::_convertIntoGlTFAxisSystem(const Matrix3 &max_transform_) {
  return get_transform_to_max_axis_system() * max_transform_ *
         get_transform_to_max_axis_system_inverse();
}

Point3 exporter_impl::_convertIntoGlTFAxisSystem(const Point3 &max_point_) {
  Matrix3 mat(TRUE);
  mat.SetTrans(max_point_);
  mat = _convertIntoGlTFAxisSystem(mat);
  auto transnew = mat.GetTrans();
  return transnew;
}

Quat exporter_impl::_convertIntoGlTFAxisSystem(const Quat &max_rotation_) {
  float angle = 0;
  Point3 axis;
  AngAxisFromQ(max_rotation_, &angle, axis);
  auto glAxis = _convertIntoGlTFAxisSystem(axis);
  // glTF use right-hand rule, Max use left-hand.
  auto glAngle = -angle;
  Quat q;
  q.Set(AngAxis(glAxis, glAngle));
  return q;
}

/// <summary>
/// <c>quat_</c> shall be normalized.
/// </summary>
Quat exporter_impl::_igameToGlTF(const Quat &quat_) {
  // glTF use right-hand rule, iGame(Max) use left-hand.
  Quat result(quat_);
  result.w = -result.w;
  return result;
}

std::u8string exporter_impl::_convertMaxName(const MSTR &max_name_) {
  return mchar_to_u8(
      std::basic_string_view<MCHAR>(max_name_, max_name_.Length()));
}

std::filesystem::path exporter_impl::_convertMaxPath(const MSTR &max_path_) {
  return std::filesystem::path(
      std::basic_string_view<MCHAR>(max_path_, max_path_.Length()));
}

Matrix3 exporter_impl::_getLocalNodeTransformMatrix(INode &max_node_,
                                                    TimeValue time_) {
  auto worldTM = max_node_.GetNodeTM(time_);
  if (!max_node_.GetParentNode()->IsRootNode()) {
    auto inverseParent = Inverse(max_node_.GetParentNode()->GetNodeTM(time_));
    return worldTM * inverseParent;
  } else {
    return worldTM;
  }
}

glm::mat4 exporter_impl::_getObjectOffsetTransformMatrix(IGameNode &igame_node_,
                                                         TimeValue time_) {
  auto &maxNode = *igame_node_.GetMaxNode();
  auto maxPos = maxNode.GetObjOffsetPos();
  auto maxScale = maxNode.GetObjOffsetScale().s;
  auto maxRot = maxNode.GetObjOffsetRot();
  /*auto maxTrans = _composeTRS(_toGLM(maxPos), _toGLM(maxRot),
  _toGLM(maxScale)); return _maxToGLTF(maxTrans);*/
  auto trans =
      _composeTRS(_mathconv_max_to_glTF::convert_point(_toGLM(maxPos)),
                  _mathconv_max_to_glTF::convert_rotation(_toGLM(maxRot)),
                  _mathconv_max_to_glTF::convert_scale(_toGLM(maxScale)));
  return trans;
}

Point3 exporter_impl::_transformPoint(const GMatrix &matrix_,
                                      const Point3 &point_) {
  return point_ * matrix_;
}

Point3 exporter_impl::_transformVector(const GMatrix &matrix_,
                                       const Point3 &point_) {
  auto inversed = GMatrix(matrix_).Inverse();

  GMatrix inverseTransposed;
  for (int r = 0; r < 4; ++r) {
    inverseTransposed.SetRow(r, Point4(inversed.GetColumn(r)));
  }

  return point_ * inverseTransposed;
}

glm::mat4
exporter_impl::_calculateLocalMatrix(glTF::object_ptr<glTF::node> node_) {
  if (!node_->is_trs()) {
    return glm::make_mat4(node_->matrix().data());
  } else {
    auto position = glm::make_vec3(node_->position().data());
    auto scale = glm::make_vec3(node_->scale().data());
    auto rotation = glm::make_quat(node_->rotation().data());
    return _composeTRS(position, rotation, scale);
  }
}

glm::mat4
exporter_impl::_calculateWorldMatrix(glTF::object_ptr<glTF::node> node_) {
  auto localTM = _calculateLocalMatrix(node_);
  if (auto parent = node_->parent()) {
    return _calculateWorldMatrix(parent) * localTM;
  } else {
    return localTM;
  }
}

exporter_impl::_trs exporter_impl::_decomposeTRS(const glm::mat4 &matrix_) {
  auto translation = glm::zero<glm::vec3>();
  glm::vec3 scale(1, 1, 1);
  auto rotation = glm::identity<glm::quat>();
  auto skew = glm::zero<glm::vec3>();
  auto perspective = glm::zero<glm::vec4>();
  auto decomposeresult =
      glm::decompose(matrix_, scale, rotation, translation, skew, perspective);
  if (!decomposeresult) {
    // TODO
  }
  return {translation, rotation, scale};
}

glm::mat4 exporter_impl::_composeTRS(const glm::vec3 &translation_,
                                     const glm::quat &rotation_,
                                     const glm::vec3 &scale_) {
  return glm::translate(translation_) * glm::mat4_cast(rotation_) *
         glm::scale(scale_);
}

glm::mat4 exporter_impl::_toGLM(const GMatrix &matrix_) {
  // glm expect the matrices to be column vector and stored in column majar;
  // In 3ds Max, all vectors are assumed to be row vectors.
  glm::mat4 result;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      // We actually done the transpose: it should have been `out[c * 4 +
      // r]`(column major).
      glm::value_ptr(result)[r * 4 + c] = matrix_.GetRow(r)[c];
    }
  }
  return result;
}

glm::vec3 exporter_impl::_toGLM(const Point3 &point_) {
  return glm::vec3(point_.x, point_.y, point_.z);
}

glm::quat exporter_impl::_toGLM(const Quat &quat_) {
  return glm::quat(quat_.w, quat_.x, quat_.y, quat_.z);
}

glm::vec3
exporter_impl::_mathconv_max_to_glTF::convert_point(const glm::vec3 &point_) {
  return glm::vec3(point_.x, point_.z, -point_.y);
}

glm::vec3
exporter_impl::_mathconv_max_to_glTF::convert_scale(const glm::vec3 &scale_) {
  // Note: the sign never change.
  return glm::vec3(scale_.x, scale_.z, scale_.y);
}

glm::quat
exporter_impl::_mathconv_max_to_glTF::convert_rotation(const glm::quat &quat_) {
  // assert(glm::length(quat_) == 1);
  // glTF use right-hand rule, iGame(Max) use left-hand.
  auto angle = glm::angle(quat_);
  auto axis = glm::axis(quat_);
  return glm::angleAxis(-angle, convert_point(axis));
}

glm::mat4 exporter_impl::_mathconv_max_to_glTF::convert_transform(
    const glm::mat4 &matrix_) {
  auto maxToGLTF = glm::rotate(
      glm::radians(static_cast<glm::mat4::value_type>(90)), glm::vec3(1, 0, 0));
  return glm::inverse(maxToGLTF) * matrix_ * maxToGLTF;
}

glm::quat exporter_impl::_mathconv_igame_to_glTF::convert_rotation(
    const glm::quat &quat_) {
  // assert(glm::length(quat_) == 1);
  // glTF use right-hand rule, iGame(Max) use left-hand.
  auto result = quat_;
  result.w = -result.w;
  return result;
}
} // namespace apricot