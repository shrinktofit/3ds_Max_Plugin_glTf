
#pragma once

#include <Max.h>
#include <impexp.h>
#include <iparamb2.h>
#include <memory>

namespace plugin {
class glTF_exporter : public SceneExport {
public:
  class class_description;

  glTF_exporter();

  ~glTF_exporter();

  virtual int ExtCount();

  virtual const MCHAR *Ext(int n);

  virtual const MCHAR *LongDesc();

  virtual const MCHAR *ShortDesc();

  virtual const MCHAR *AuthorName();

  virtual const MCHAR *CopyrightMessage();

  virtual const MCHAR *OtherMessage1();

  virtual const MCHAR *OtherMessage2();

  virtual unsigned int Version();

  virtual void ShowAbout(HWND hWnd);

  virtual int DoExport(const MCHAR *name,
                       ExpInterface *ei,
                       Interface *i,
                       BOOL suppressPrompts = FALSE,
                       DWORD options = 0);

  virtual BOOL SupportsOptions(int ext, DWORD options);

private:
  class _impl_t;

  std::unique_ptr<_impl_t> _impl;
};

class glTF_exporter::class_description : public ClassDesc2 {
public:
  virtual int IsPublic() {
    return 1;
  }

  virtual void *Create(BOOL /*loading = FALSE*/);

  virtual const MCHAR *ClassName();

  virtual SClass_ID SuperClassID() {
    return SCENE_EXPORT_CLASS_ID;
  }

  virtual Class_ID ClassID();

  virtual const MCHAR *Category();

  virtual const MCHAR *InternalName();

  virtual HINSTANCE HInstance();
};
} // namespace plugin