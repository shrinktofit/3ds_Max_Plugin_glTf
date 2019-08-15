

#include <Windows.h>
#include <cassert>
#include <memory>
#include <stdexcept>
#include <fant/support/win32/mchar_to_utf8.h>

namespace fant::win32 {
std::u8string mchar_to_utf8(std::basic_string_view<MCHAR> mstr_) {
  // https://stackoverflow.com/questions/215963/how-do-you-properly-use-widechartomultibyte
  if (mstr_.size() > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("String too long.");
  }
  auto szMStr = static_cast<int>(mstr_.size());

  auto szResultStr = WideCharToMultiByte(CP_UTF8, 0, mstr_.data(), szMStr, NULL,
                                         0, NULL, NULL);
  std::u8string result(szResultStr, 0);
  (void)WideCharToMultiByte(CP_UTF8, 0, mstr_.data(), szMStr, reinterpret_cast<char*>(result.data()),
                            szResultStr, NULL, NULL);
  return result;
}
} // namespace fant::win32