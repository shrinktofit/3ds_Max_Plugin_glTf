

#pragma once

#include <fant/3ds_Max_classes/export_settings.h>
#include <optional>

namespace fant {
std::optional<export_settings> open_export_dialog();
} // namespace fant