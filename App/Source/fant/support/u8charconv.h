
#pragma once

#include <charconv>

namespace fant {
struct to_u8chars_result {
  char8_t *ptr;
  std::errc ec;
};

template <typename Integer>
to_u8chars_result to_u8chars(char8_t *first_,
                             char8_t *const last_,
                             const Integer value_,
                             const int base_ = 10) {
  static_assert(
      '0' ==
      u8'0'); // Ensure the execution character set is compactable with UTF-8.
  auto tocharsResult =
      std::to_chars(reinterpret_cast<char *>(first_),
                    reinterpret_cast<char *const>(last_), value_, base_);
  return {reinterpret_cast<char8_t *>(tocharsResult.ptr), tocharsResult.ec};
}
} // namespace fant