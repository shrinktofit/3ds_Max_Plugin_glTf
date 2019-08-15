
#pragma once

#include <Windows.h>

namespace fant::win32 {
HINSTANCE get_instance();

void set_instance(HINSTANCE handle_);
} // namespace fant::win32
