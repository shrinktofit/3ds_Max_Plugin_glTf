Add-Type -AssemblyName "$($PSScriptRoot)/../bin/Debug/netstandard2.0/Apricot.Ui.dll"

$dialog = New-Object Apricot.Ui.ExportDialog
$dialog.Show()
Write-Host $dialog.Result