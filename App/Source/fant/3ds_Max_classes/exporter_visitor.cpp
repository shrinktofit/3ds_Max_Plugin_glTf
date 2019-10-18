
#include <IGame/IGame.h>
#include <cppcodec/base64_rfc4648.hpp>
#include <fant/3ds_Max_classes/exporter_visitor.h>
#include <filesystem>
#include <functional>

namespace fant {
exporter_visitor::exporter_visitor(Interface &max_interface_,
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
    auto nFrame =
        static_cast<glTF::integer>(std::ceil(animRange.Duration() / step));
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

  for (auto &nodeMap : _nodeMaps2) {
    auto &igameNode = *nodeMap.first;
    auto &glTFNode = nodeMap.second;
    auto object = igameNode.GetIGameObject();
    switch (object->GetIGameType()) {
    case IGameObject::ObjectTypes::IGAME_MESH: {
      auto &igameMesh = static_cast<IGameMesh &>(*object);
      if (auto immMesh = _exportMesh(igameNode, igameMesh)) {
        glTF::object_ptr<glTF::skin> glTFSkin;
        if (igameMesh.IsObjectSkinned()) {
          /*glTFSkin =
              _exportSkin(igameNode, *igameMesh.GetIGameSkin(), *immMesh);*/
        }
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

GMatrix fant::exporter_visitor::_convertIntoGlTFAxisSystem(
    const GMatrix &max_transform_) {
  return GMatrix(get_transform_to_max_axis_system()) * max_transform_ *
         GMatrix(get_transform_to_max_axis_system_inverse());
}

Matrix3
exporter_visitor::_convertIntoGlTFAxisSystem(const Matrix3 &max_transform_) {
  return get_transform_to_max_axis_system() * max_transform_ *
         get_transform_to_max_axis_system_inverse();
}

Point3 exporter_visitor::_convertIntoGlTFAxisSystem(const Point3 &max_point_) {
  Matrix3 mat(TRUE);
  mat.SetTrans(max_point_);
  mat = _convertIntoGlTFAxisSystem(mat);
  auto transnew = mat.GetTrans();
  return transnew;
}

Quat exporter_visitor::_convertIntoGlTFAxisSystem(const Quat &max_rotation_) {
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

std::u8string exporter_visitor::_convertMaxName(const MSTR &max_name_) {
  return win32::mchar_to_utf8(
      std::basic_string_view<MCHAR>(max_name_, max_name_.Length()));
}

std::filesystem::path exporter_visitor::_convertMaxPath(const MSTR &max_path_) {
  return std::filesystem::path(
      std::basic_string_view<MCHAR>(max_path_, max_path_.Length()));
}

Matrix3 exporter_visitor::_getLocalNodeTransformMatrix(INode &max_node_,
                                                       TimeValue time_) {
  auto worldTM = max_node_.GetNodeTM(time_);
  if (!max_node_.GetParentNode()->IsRootNode()) {
    auto inverseParent = Inverse(max_node_.GetParentNode()->GetNodeTM(time_));
    return worldTM * inverseParent;
  } else {
    return worldTM;
  }
}

GMatrix
exporter_visitor::_getObjectOffsetTransformMatrix(IGameNode &igame_node_,
                                                  TimeValue time_) {
  auto objectOffsetWorld = igame_node_.GetObjectTM(time_);
  auto invObjectdOffsetWorld = objectOffsetWorld.Inverse();
  auto worldTM = igame_node_.GetWorldTM(time_);
  return invObjectdOffsetWorld * worldTM;
}

Point3 exporter_visitor::_transformPoint(const GMatrix &matrix_,
                                         const Point3 &point_) {
  return point_ * matrix_;
}

Point3 exporter_visitor::_transformVector(const GMatrix &matrix_,
                                          const Point3 &point_) {
  auto inversed = GMatrix(matrix_).Inverse();

  GMatrix inverseTransposed;
  for (int r = 0; r < 4; ++r) {
    inverseTransposed.SetRow(r, Point4(inversed.GetColumn(r)));
  }

  return point_ * inverseTransposed;
}
} // namespace fant