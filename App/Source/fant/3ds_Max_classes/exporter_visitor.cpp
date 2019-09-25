
#include <fant/3ds_Max_classes/exporter_visitor.h>

namespace fant {
exporter_visitor::exporter_visitor(Interface &max_interface_,
                                   glTF::document &document_,
                                   const export_settings &settings_)
    : _maxInterface(max_interface_), _document(document_),
      _glTFScene(_document.factory().make<glTF::scene>()),
      _settings(settings_) {
  _document.default_scene(_glTFScene);
  _mainBuffer = _document.factory().make<glTF::buffer>();

  auto animRange = _maxInterface.GetAnimRange();
  auto fps = 30;
  if (animRange.Duration()) {
    auto step = SecToTicks(fps);
    auto nFrame =
        static_cast<glTF::integer>(std::ceil(animRange.Duration() / step));
    auto inputAccessor =
        _makeSimpleAccessor(glTF::accessor::type_type::scalar,
                            glTF::accessor::component_type::the_float, nFrame);
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
}

int exporter_visitor::callback(INode *max_node_) {
  auto glTFNode = _convertNode(*max_node_);

  if (!max_node_->IsRootNode()) {
    auto parentMaxNode = max_node_->GetParentNode();
    auto rParentGlTfNode = _nodeMaps.find(parentMaxNode);
    if (rParentGlTfNode != _nodeMaps.end()) {
      rParentGlTfNode->second->add_child(glTFNode);
    } else {
      _glTFScene->add_node(glTFNode);
    }
  }

  auto object = max_node_->EvalWorldState(0).obj;
  if (object->CanConvertToType({TRIOBJ_CLASS_ID, 0})) {
    auto triObject = static_cast<TriObject *>(
        object->ConvertToType(0, {TRIOBJ_CLASS_ID, 0}));
    if (triObject) {
      auto glTFMesh = _convertTriObj(*triObject);
      glTFNode->mesh(glTFMesh);
      if (triObject != object) {
        triObject->DeleteMe();
      }
    }
  }

  auto objectRef = max_node_->GetObjectRef();
  if (objectRef->SuperClassID() == GEN_DERIVOB_CLASS_ID) {
    auto derivedObject = reinterpret_cast<IDerivedObject *>(objectRef);
    const auto numModifiers = derivedObject->NumModifiers();
    for (std::remove_const_t<decltype(numModifiers)> iModifier = 0;
         iModifier < numModifiers; ++iModifier) {
      auto modifier = derivedObject->GetModifier(iModifier);
      auto skin = reinterpret_cast<ISkin *>(modifier->GetInterface(I_SKIN));
      if (!skin) {
        continue;
      }

      auto skinContextData = skin->GetContextInterface(max_node_);
      if (!skinContextData) {
        continue;
      }

      const auto numPoints = skinContextData->GetNumPoints();
      for (std::remove_const_t<decltype(numPoints)> iPoint = 0;
           iPoint < numPoints; ++iPoint) {
        const auto numAffectBones =
            skinContextData->GetNumAssignedBones(iPoint);
        for (std::remove_const_t<decltype(numAffectBones)> iAffectBone = 0;
             iAffectBone != numAffectBones; ++iAffectBone) {
          auto affectBoneIndex =
              skinContextData->GetAssignedBone(iPoint, iAffectBone);
          auto weight = skinContextData->GetBoneWeight(iPoint, iAffectBone);
        }
      }

      const auto numBones = skin->GetNumBones();
      for (std::remove_const_t<decltype(numBones)> iBone = 0; iBone < numBones;
           ++iBone) {
        auto boneProperty = skin->GetBoneProperty(iBone);
      }
    }
  }

  if (_animBaking) {
    _bakeAnimation(*max_node_, _animBaking->start, _animBaking->step,
                   _animBaking->frame_count, glTFNode, _animBaking->input);
  }

  _nodeMaps.emplace(max_node_, glTFNode);
  // _readAnimation(*max_node_);
  return TREE_CONTINUE;
}
} // namespace fant