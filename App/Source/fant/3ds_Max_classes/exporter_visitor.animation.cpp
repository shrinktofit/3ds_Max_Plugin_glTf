
#include <fant/3ds_Max_classes/exporter_visitor.h>
#include <variant>

namespace fant {
void exporter_visitor::_bakeAnimation(INode &max_node_,
                                      TimeValue start_time_,
                                      TimeValue step_,
                                      TimeValue frame_count_,
                                      glTF::object_ptr<glTF::node> glTF_node_,
                                      glTF::object_ptr<glTF::accessor> input_) {
  if (frame_count_ == 0) {
    return;
  }

  std::vector<Matrix3> localNodeTMs(frame_count_);
  for (decltype(frame_count_) iFrame = 0; iFrame < frame_count_; ++iFrame) {
    localNodeTMs[iFrame] =
        _getLocalNodeTransformMatrix(max_node_, start_time_ + step_ * iFrame);
  }

  // If this node contains same matrix at every frame. We do not process
  // further.
  if (std::all_of(localNodeTMs.begin() + 1, localNodeTMs.end(),
                  [&](const auto &localNodeTM) {
                    return localNodeTM == localNodeTMs.front();
                  })) {
    return;
  }

  std::vector<Point3> positions(frame_count_);
  std::vector<Quat> rotations(frame_count_);
  std::vector<Point3> scales(frame_count_);
  for (decltype(localNodeTMs.size()) iTM = 0; iTM < localNodeTMs.size();
       ++iTM) {
    Point3 t;
    Quat r;
    Point3 s;
    DecomposeMatrix(localNodeTMs[iTM], t, r, s);
    positions[iTM] = _convertIntoGlTFAxisSystem(t);
    rotations[iTM] = _convertIntoGlTFAxisSystem(r);
    scales[iTM] = _convertIntoGlTFAxisSystem(s);
  }

  if (!std::all_of(positions.begin() + 1, positions.end(),
                   [&](const auto &position_) {
                     return position_ == positions.front();
                   })) {
    auto accessor = _makeSimpleAccessor(gsl::make_span(positions));
    auto sampler = _mainAnimation->factory().make<glTF::animation::sampler>(
        input_, accessor);
    _mainAnimation->factory().make<glTF::animation::channel>(
        sampler,
        glTF::animation::channel::target_type{
            glTF::animation::channel::target_type::builtin_path::translation,
            glTF_node_});
  }

  if (!std::all_of(rotations.begin() + 1, rotations.end(),
                   [&](const auto &rotation_) {
                     return rotation_ == rotations.front();
                   })) {
    auto accessor = _makeSimpleAccessor(gsl::make_span(rotations));
    auto sampler = _mainAnimation->factory().make<glTF::animation::sampler>(
        input_, accessor);
    _mainAnimation->factory().make<glTF::animation::channel>(
        sampler,
        glTF::animation::channel::target_type{
            glTF::animation::channel::target_type::builtin_path::rotation,
            glTF_node_});
  }

  if (!std::all_of(scales.begin() + 1, scales.end(), [&](const auto &scale_) {
        return scale_ == scales.front();
      })) {
    auto accessor = _makeSimpleAccessor(gsl::make_span(scales));
    auto sampler = _mainAnimation->factory().make<glTF::animation::sampler>(
        input_, accessor);
    _mainAnimation->factory().make<glTF::animation::channel>(
        sampler, glTF::animation::channel::target_type{
                     glTF::animation::channel::target_type::builtin_path::scale,
                     glTF_node_});
  }
}
} // namespace fant