
#pragma once

#include <apricot/utilities/u8charconv.h>
#include <array>
#include <cassert>
#include <string>

namespace apricot {
template <typename Integer>
std::u8string to_u8string(const Integer value_, int base_ = 10) {
  std::array<char8_t, sizeof(Integer) * CHAR_BIT + 1> buffer;
  auto [ptr, ec] =
      to_u8chars(buffer.data(), buffer.data() + buffer.size(), value_, base_);
  assert(ec == std::errc());
  return {buffer.data(), ptr};
}
} // namespace apricot