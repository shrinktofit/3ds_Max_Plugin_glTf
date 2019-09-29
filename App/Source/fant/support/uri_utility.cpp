
#include <fant/support/uri_utility.h>
#include <filesystem>
#include <sstream>

namespace fant {
std::u8string uri_from_path(const std::filesystem::path &path_) {
  // https://blogs.msdn.microsoft.com/ie/2006/12/06/file-uris-in-windows/
  namespace fs = std::filesystem;
  auto normalized = path_.lexically_normal();
  std::basic_stringstream<char8_t> result;
  if (normalized.is_absolute()) {
    result << u8"file://";
    if (normalized.has_root_name()) {
      result << u8"/";
    }
  } else {
    result << u8"./";
  }
  // TODO: uri encoding
  result << path_.generic_u8string();
  return result.str();
}
} // namespace fant