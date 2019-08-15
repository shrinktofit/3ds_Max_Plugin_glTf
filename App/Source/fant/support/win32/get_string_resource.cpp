
#include <Windows.h>
#include <fant/support/win32/get_string_resource.h>
#include <fant/support/win32/instance.h>

namespace fant::win32 {
MCHAR *get_string_resource(int id_) {
  static MCHAR buf[1024];
  auto hInstance = get_instance();
  if (hInstance)
    return LoadString(hInstance, id_, buf, sizeof(buf)) ? buf : nullptr;
  return nullptr;
}
} // namespace fant::win32
