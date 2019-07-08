
#include <tapu/support/win32/instance.h>

namespace tapu::win32 {
HINSTANCE global_instance = nullptr;

HINSTANCE get_instance() {
  return global_instance;
}

void set_instance(HINSTANCE handle_) {
  global_instance = handle_;
}
} // namespace tapu::win32