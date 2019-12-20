
#pragma once

#include <optional>

namespace apricot {
struct export_settings {
  enum class format_t {
    json,
    binary,
    embedded,
  } format = format_t::json;

  enum class image_storage_t {
    standalone,
    embedded_as_binary,
    embedded_as_uri,
  } image_storage;

  enum class index_type_t {
    u8,
    u16,
    u32,
    least_u8,
    least_u16,
  } index_type;

  using max_joint_influence_t = unsigned;
  std::optional<max_joint_influence_t> max_joint_influence;
};
} // namespace apricot
