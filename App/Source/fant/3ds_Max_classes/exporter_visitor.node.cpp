
#include <fant/3ds_Max_classes/exporter_visitor.h>

namespace fant {
glTF::object_ptr<glTF::node> exporter_visitor::_convertNode(INode &max_node_) {
  auto maxNodeName = std::basic_string_view<MCHAR>(max_node_.GetName());
  auto name = win32::mchar_to_utf8(maxNodeName);

  auto nodeTM = max_node_.GetNodeTM(0);
  Matrix3 nodeLocalTM(1);
  if (max_node_.IsRootNode()) {
    nodeLocalTM = nodeTM;
  } else {
    auto parentNodeTM = max_node_.GetParentNode()->GetNodeTM(0);
    nodeLocalTM = nodeTM * Inverse(parentNodeTM);
  }

  auto objectOffsetTM = _getObjectOffsetTM(max_node_);

  auto objectLocalTM = objectOffsetTM * nodeLocalTM;
  auto trans = objectLocalTM.GetTrans();
  Point3 pos;
  Point3 scale;
  Quat rot;
  DecomposeMatrix(objectLocalTM, pos, rot, scale);

  auto glTFNode = _document.factory().make<glTF::node>();
  glTFNode->name(name);

  if (pos != Point3::Origin) {
    glTFNode->set_position(pos.x, pos.y, pos.z);
  }

  if (scale != Point3(1, 1, 1)) {
    glTFNode->set_scale(scale.x, scale.y, scale.z);
  }

  if (!rot.IsIdentity()) {
    rot.Normalize();
    glTFNode->set_rotation(rot.x, rot.y, rot.z, rot.w);
  }

  return glTFNode;
}
} // namespace fant