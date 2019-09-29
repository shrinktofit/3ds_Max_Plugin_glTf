
#pragma once

namespace fant {
struct export_settings {
  enum class index_type_setting {
    unsigned_8,
    unsigned_16,
    unsigned_32,
  } index_type;

  enum class index_setting {
    fixed,
    at_least,
  } index;
};
} // namespace fant
