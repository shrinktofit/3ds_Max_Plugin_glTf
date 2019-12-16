

### launch.vs.json

```json
{
  "version": "0.2.1",
  "defaults": {},
  "configurations": [
    {
      "type": "dll",
      "project": "CMakeLists.txt",
      "projectTarget": "Apricot.dll (Apricot\\Debug\\Apricot.dll)",
      "name": "Apricot.dll (Apricot\\Debug\\Apricot.dll)",
      "exe": "D:\\Program Files\\Autodesk\\3ds Max 2019\\3dsmax.exe",
      "args": [
        "-silent",
        "-U",
        "MAXScript",
        "X:/ExportTest.ms"
      ]
    }
  ]
}
```

### MaxScript Template

--loadMaxFile "path-to-max-file.max"
--exportFile "path-to-glTF-file.GLTF" #noPrompt

### Install plugin

Launch 3ds Max, go to Customize → Configure System Paths... → 3rd Party Plug-Ins, click Add... and choose the C:\Program Files\Autodesk\3ds Max 2017\plugins\appleseed folder.