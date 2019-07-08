$maxVersionNumber = "21.0"
$maxRegistryKey = 'HKLM:\SOFTWARE\Autodesk\3dsMax'

$maxInstallDir = (Get-ItemProperty -Path "$maxRegistryKey\$maxVersionNumber" -Name InstallDir).InstallDir
Write-Host "3ds Max install directory: $maxInstallDir"

$pluginDir = "$maxInstallDir\plugins"
if (-Not (Test-Path $pluginDir)) {
    Write-Error "plugin directory $pluginDir doesn't exists."
    return -1
} else {
    Write-Host "3ds Max plugin directory: $pluginDir"
}

$myPluginDir = ".\out\build\x64-Debug\App\Debug"
if (-Not (Test-Path $myPluginDir)) {
    Write-Error "Cannot find target plugin at $myPluginDir , Please build it first."
}

Copy-Item -Path "$myPluginDir\3ds_Max_Plugin_glTf.dle" -Destination "$pluginDir" -Verbose
Copy-Item -Path "$myPluginDir\3ds_Max_Plugin_glTf.pdb" -Destination "$pluginDir" -Verbose
