
#include <fant/3ds_Max_classes/exporter_visitor.h>
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

  class TreeEnumFunctor : public ITreeEnumProc {
  public:
    TreeEnumFunctor(std::function<int(INode *)> fx_) : _fx(fx_) {
    }

    int callback(INode *max_node_) {
      return _fx(max_node_);
    }

  protected:
    std::function<int(INode *)> _fx;
  };

  TreeEnumFunctor constructSceneGraph(
      std::bind(&exporter_visitor::_constructSceneGraphProc, this,
                std::placeholders::_1));
  scene_.EnumTree(&constructSceneGraph);

  TreeEnumFunctor mainProc(
      std::bind(&exporter_visitor::_mainProc, this, std::placeholders::_1));
  scene_.EnumTree(&mainProc);

  if (_document.factory().get_size<glTF::buffer>() == 0) {
    auto demandBuffer = _document.factory().make<glTF::buffer>();
    demandBuffer->allocate(1, 0);
  }
}

int exporter_visitor::_mainProc(INode *max_node_) {
  auto rglTFNode = _nodeMaps.find(max_node_);
  if (rglTFNode == _nodeMaps.end()) {
    assert(false);
    return TREE_CONTINUE;
  }

  auto glTFNode = rglTFNode->second;

  auto objectOffsetTransformNode =
      _trySimulateObjectOffsetTransform(*max_node_);
  if (objectOffsetTransformNode) {
    glTFNode->add_child(objectOffsetTransformNode);
  }

  auto actualNode =
      objectOffsetTransformNode ? objectOffsetTransformNode : glTFNode;

  auto immMesh = _tryExportMesh(*max_node_);
  if (immMesh) {
    auto glTFSkin = _tryExportSkin(*max_node_, *immMesh);
    auto glTFMesh = _convertMesh(*immMesh);
    actualNode->mesh(glTFMesh);
    if (glTFSkin) {
      actualNode->skin(glTFSkin);
    }
  }

  if (_animBaking) {
    _bakeAnimation(*max_node_, _animBaking->start, _animBaking->step,
                   _animBaking->frame_count, glTFNode, _animBaking->input);
  }

  return TREE_CONTINUE;
}
} // namespace fant