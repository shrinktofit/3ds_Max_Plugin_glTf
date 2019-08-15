

#pragma once

#include <strbasic.h>
#include <string>
#include <string_view>

namespace fant::win32 {
std::u8string mchar_to_utf8(std::basic_string_view<MCHAR> mstr_);
}