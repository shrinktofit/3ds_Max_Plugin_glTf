
#pragma once

#include <Windows.h>

namespace tapu::win32 {
HINSTANCE get_instance();

void set_instance(HINSTANCE handle_);
} // namespace tapu::win32