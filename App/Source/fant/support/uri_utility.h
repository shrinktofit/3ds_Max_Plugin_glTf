
#pragma once

#include <filesystem>
#include <string>

namespace fant {
std::u8string uri_from_path(const std::filesystem::path &path_);
}