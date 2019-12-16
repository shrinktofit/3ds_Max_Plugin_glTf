
#pragma once

#include <Max.h>
#include <apricot/api-exports.h>
#include <impexp.h>
#include <iparamb2.h>

namespace apricot {
class APRICOT_API exporter {
public:
  int do_export(const MCHAR *name,
                ExpInterface *ei,
                Interface *i,
                BOOL suppressPrompts = FALSE,
                DWORD options = 0);

  BOOL supports_options(int ext, DWORD options);
};
} // namespace apricot