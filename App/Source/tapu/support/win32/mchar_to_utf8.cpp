

#define NOMINMAX
#include <Windows.h>
#include <cassert>
#include <memory>
#include <stdexcept>
#include <tapu/support/win32/mchar_to_utf8.h>

namespace tapu::win32 {
std::string mchar_to_utf8(std::basic_string_view<MCHAR> mstr_) {
  // https://stackoverflow.com/questions/215963/how-do-you-properly-use-widechartomultibyte
  if (mstr_.size() > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("String too long.");
  }
  auto szMStr = static_cast<int>(mstr_.size());

  auto szResultStr = WideCharToMultiByte(CP_UTF8, 0, mstr_.data(), szMStr, NULL,
                                         0, NULL, NULL);
  std::string result(szResultStr, 0);
  (void)WideCharToMultiByte(CP_UTF8, 0, mstr_.data(), szMStr, result.data(),
                            szResultStr, NULL, NULL);
  return result;
}
} // namespace tapu::win32