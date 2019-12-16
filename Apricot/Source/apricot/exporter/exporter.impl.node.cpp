
#include <apricot/exporter/exporter.impl.h>
#include <glm/gtc/constants.hpp>

namespace apricot {
void exporter_impl::_setTrs(glTF::node &node_, const GMatrix &matrix_) {
  // Babylon exporter says that "use babylon decomposition, as 3ds max built-in
  // values are no correct". For some model, it's true: the `matrix_.Rotation()`
  // will return quat(0, vec3(1, 0, 0)) for non-rotated node.
  auto glmTM = _toGLM(matrix_);
  auto [translation, rotation, scale] = _decomposeTRS(glmTM);

  if (translation != glm::zero<glm::vec3>()) {
    node_.position({translation.x, translation.y, translation.z});
  }

  if (scale != glm::vec3(1, 1, 1)) {
    node_.scale({scale.x, scale.y, scale.z});
  }

  if (rotation != glm::identity<glm::quat>()) {
    auto normr = glm::normalize(rotation);
    normr = _mathconv_igame_to_glTF::convert_rotation(normr);
    node_.rotation({normr.x, normr.y, normr.z, normr.w});
  }
}

glTF::object_ptr<glTF::node>
exporter_impl::_exportNode(IGameNode &igame_node_) {
  auto glTFNode = _document.factory().make<glTF::node>();

  auto name = _convertMaxName(igame_node_.GetName());
  glTFNode->name(name);
  auto namen = reinterpret_cast<const char *>(name.c_str());

  _setTrs(*glTFNode, igame_node_.GetLocalTM(0));

  for (decltype(igame_node_.GetChildCount()) iChild = 0;
       iChild < igame_node_.GetChildCount(); ++iChild) {
    auto childGlTFNode = _exportNode(*igame_node_.GetNodeChild(iChild));
    glTFNode->add_child(childGlTFNode);
  }

  _nodeMaps2.emplace(&igame_node_, glTFNode);
  return glTFNode;
}

Matrix3 exporter_impl::_calcOffsetTransformMatrix(INode &max_node_) {
  Matrix3 tm(1);
  Point3 pos = max_node_.GetObjOffsetPos();
  tm.PreTranslate(pos);
  Quat quat = max_node_.GetObjOffsetRot();
  PreRotateMatrix(tm, quat);
  ScaleValue scaleValue = max_node_.GetObjOffsetScale();
  ApplyScaling(tm, scaleValue);
  return tm;
}
} // namespace fant