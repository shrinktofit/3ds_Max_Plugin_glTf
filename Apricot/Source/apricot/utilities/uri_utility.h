
#pragma once

#include <filesystem>
#include <string>

namespace apricot {
std::u8string uri_from_path(const std::filesystem::path &path_);
}