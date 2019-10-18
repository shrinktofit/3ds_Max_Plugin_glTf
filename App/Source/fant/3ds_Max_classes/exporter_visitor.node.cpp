
#include <fant/3ds_Max_classes/exporter_visitor.h>

namespace fant {
void exporter_visitor::_setTrs(glTF::node &node_,
                               const Point3 &t_,
                               const Quat &r_,
                               const Point3 &s_) {
  if (t_ != Point3::Origin) {
    node_.set_position(t_.x, t_.y, t_.z);
  }

  if (s_ != Point3(1, 1, 1)) {
    node_.set_scale(s_.x, s_.y, s_.z);
  }

  if (!r_.IsIdentity()) {
    auto normr = Quat(r_);
    normr.Normalize();
    node_.set_rotation(normr.x, normr.y, normr.z, normr.w);
  }
}

void exporter_visitor::_setTrs(glTF::node &node_, const Matrix3 &matrix_) {
  Point3 t;
  Point3 s;
  Quat r;
  DecomposeMatrix(matrix_, t, r, s);
  _setTrs(node_, t, r, s);
}

void exporter_visitor::_setTrs(glTF::node &node_, const GMatrix &matrix_) {
  _setTrs(node_, (matrix_.Translation()), (matrix_.Rotation()),
          (matrix_.Scaling()));
  return;
  _setTrs(node_, _convertIntoGlTFAxisSystem(matrix_.Translation()),
          _convertIntoGlTFAxisSystem(matrix_.Rotation()),
          _convertIntoGlTFAxisSystem(matrix_.Scaling()));
}

glTF::object_ptr<glTF::node> exporter_visitor::_convertNode(INode &max_node_) {
  auto maxNodeName = std::basic_string_view<MCHAR>(max_node_.GetName());
  auto name = win32::mchar_to_utf8(maxNodeName);
  auto localNodeTM = _getLocalNodeTransformMatrix(max_node_, 0);
  auto glTFNode = _document.factory().make<glTF::node>();
  glTFNode->name(name);
  _setTrs(*glTFNode, _convertIntoGlTFAxisSystem(localNodeTM));
  return glTFNode;
}

glTF::object_ptr<glTF::node>
exporter_visitor::_exportNode(IGameNode &igame_node_) {
  auto glTFNode = _document.factory().make<glTF::node>();

  auto name = _convertMaxName(igame_node_.GetName());
  glTFNode->name(name);
  auto namen = reinterpret_cast<const char*>(name.c_str());

  auto gameWTM = igame_node_.GetWorldTM(0);
  auto rawWTM = GMatrix(igame_node_.GetMaxNode()->GetNodeTM(0));

  _setTrs(*glTFNode, igame_node_.GetLocalTM(0));

  for (decltype(igame_node_.GetChildCount()) iChild = 0;
       iChild < igame_node_.GetChildCount(); ++iChild) {
    auto childGlTFNode = _exportNode(*igame_node_.GetNodeChild(iChild));
    glTFNode->add_child(childGlTFNode);
  }

  _nodeMaps2.emplace(&igame_node_, glTFNode);
  return glTFNode;
}

Matrix3 exporter_visitor::_calcOffsetTransformMatrix(INode &max_node_) {
  Matrix3 tm(1);
  Point3 pos = max_node_.GetObjOffsetPos();
  tm.PreTranslate(pos);
  Quat quat = max_node_.GetObjOffsetRot();
  PreRotateMatrix(tm, quat);
  ScaleValue scaleValue = max_node_.GetObjOffsetScale();
  ApplyScaling(tm, scaleValue);
  return tm;
}

glTF::object_ptr<glTF::node>
exporter_visitor::_trySimulateObjectOffsetTransform(INode &max_node_) {
  auto pos = max_node_.GetObjOffsetPos();
  auto rot = max_node_.GetObjOffsetRot();
  auto scaleValue = max_node_.GetObjOffsetScale();

  if (pos == Point3::Origin && rot.IsIdentity() &&
      scaleValue.s == Point3(1, 1, 1)) {
    return nullptr;
  }

  auto glTFNode = _document.factory().make<glTF::node>();
  glTFNode->name(u8"Object Offset Transform");
  _setTrs(*glTFNode, _convertIntoGlTFAxisSystem(pos),
          _convertIntoGlTFAxisSystem(rot),
          _convertIntoGlTFAxisSystem(scaleValue.s));
  return glTFNode;
}
} // namespace fant