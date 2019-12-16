
#pragma once

#include <strbasic.h>
#include <string>
#include <string_view>

namespace apricot {
std::u8string mchar_to_u8(std::basic_string_view<MCHAR> mstr_);
}