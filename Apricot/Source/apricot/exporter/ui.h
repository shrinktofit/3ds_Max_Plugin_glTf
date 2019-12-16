

#pragma once

#include <apricot/exporter/export_settings.h>
#include <optional>

namespace apricot {
std::optional<export_settings> open_export_dialog();
} // namespace apricot