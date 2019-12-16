
#include <apricot/exporter/exporter.impl.h>
#include <glm/gtc/type_ptr.hpp>
#include <variant>

namespace apricot {
void exporter_impl::_bakeAnimation(IGameNode &igame_node_,
                                      TimeValue start_time_,
                                      TimeValue step_,
                                      TimeValue frame_count_,
                                      glTF::object_ptr<glTF::node> glTF_node_,
                                      glTF::object_ptr<glTF::accessor> input_) {
  if (frame_count_ == 0) {
    return;
  }

  std::vector<GMatrix> localNodeTMs(frame_count_);
  for (decltype(frame_count_) iFrame = 0; iFrame < frame_count_; ++iFrame) {
    localNodeTMs[iFrame] = igame_node_.GetLocalTM(start_time_ + step_ * iFrame);
  }

  // If this node contains same matrix at every frame. We do not process
  // further.
  if (std::all_of(localNodeTMs.begin() + 1, localNodeTMs.end(),
                  [&](const auto &localNodeTM) {
                    return localNodeTM == localNodeTMs.front();
                  })) {
    return;
  }

  std::vector<glm::vec3> positions(frame_count_);
  std::vector<glm::quat> rotations(frame_count_);
  std::vector<glm::vec3> scales(frame_count_);
  for (decltype(localNodeTMs.size()) iTM = 0; iTM < localNodeTMs.size();
       ++iTM) {
    auto [translation, rotation, scale] =
        _decomposeTRS(_toGLM(localNodeTMs[iTM]));
    positions[iTM] = translation;
    rotations[iTM] = _mathconv_igame_to_glTF::convert_rotation(rotation);
    scales[iTM] = scale;
  }

  /// DEBUG
  /*for (decltype(localNodeTMs.size()) iTM = 0; iTM < localNodeTMs.size();
    ++iTM) {
    auto [translation, rotation, scale] =
      _decomposeTRS(_toGLM(localNodeTMs[iTM]));
    rotations[iTM] = _mathconv_igame_to_glTF::convert_rotation(rotation);
  }*/

  if (!std::all_of(positions.begin() + 1, positions.end(),
                   [&](const auto &position_) {
                     return position_ == positions.front();
                   })) {
    auto accessor = _makeSimpleAccessor(
        glTF::accessor::type_type::vec3,
        glTF::accessor::component_type::the_float,
        static_cast<glTF::accessor::size_type>(positions.size()));
    auto accessorData =
        accessor->typed_data<glTF::accessor::component_type::the_float>();
    for (decltype(positions.size()) i = 0; i < positions.size(); ++i) {
      std::copy_n(glm::value_ptr(positions[i]), 3, accessorData + 3 * i);
    }

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
    auto accessor = _makeSimpleAccessor(
        glTF::accessor::type_type::vec4,
        glTF::accessor::component_type::the_float,
        static_cast<glTF::accessor::size_type>(rotations.size()));
    auto accessorData =
        accessor->typed_data<glTF::accessor::component_type::the_float>();
    for (decltype(rotations.size()) i = 0; i < rotations.size(); ++i) {
      std::copy_n(glm::value_ptr(rotations[i]), 4, accessorData + 4 * i);
    }

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
    auto accessor = _makeSimpleAccessor(
        glTF::accessor::type_type::vec3,
        glTF::accessor::component_type::the_float,
        static_cast<glTF::accessor::size_type>(scales.size()));
    auto accessorData =
        accessor->typed_data<glTF::accessor::component_type::the_float>();
    for (decltype(scales.size()) i = 0; i < scales.size(); ++i) {
      std::copy_n(glm::value_ptr(scales[i]), 3, accessorData + 3 * i);
    }

    auto sampler = _mainAnimation->factory().make<glTF::animation::sampler>(
        input_, accessor);
    _mainAnimation->factory().make<glTF::animation::channel>(
        sampler, glTF::animation::channel::target_type{
                     glTF::animation::channel::target_type::builtin_path::scale,
                     glTF_node_});
  }
}
} // namespace apricot