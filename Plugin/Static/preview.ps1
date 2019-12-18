Add-Type -AssemblyName System.Windows.Forms

$browser = New-Object System.Windows.Forms.WebBrowser
$browser.Dock = [System.Windows.Forms.DockStyle]::Fill;

$form = New-Object System.Windows.Forms.Form
$form.Controls.Add($browser)

$indexFilePath = [System.IO.Path]::Combine([System.IO.Directory]::GetCurrentDirectory(), "export-dialog.html")
$indexUrl = New-Object System.Uri $indexFilePath
Write-Host "URL: $($indexUrl)"
$browser.Navigate($indexUrl)

$form.ShowDialog()