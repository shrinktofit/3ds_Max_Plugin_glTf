function find3dsMaxSDK {
    $sdkHomeEnvVarNames = "ADSK_3DSMAX_SDK_2020", "ADSK_3DSMAX_SDK_2019"
    foreach ($sdkHomeEnvVarName in $sdkHomeEnvVarNames) {
        $sdkHome = [System.Environment]::GetEnvironmentVariable($sdkHomeEnvVarName)
        if ($sdkHome) {
            return $sdkHome
        }
    }
    return $null
}

function find3dsMax {
    $versionNumbers = "22.0", "21.0"
    foreach ($versionNumber in $versionNumbers) {
        $registyPath = "HKLM:\SOFTWARE\Autodesk\3dsMax\$versionNumber"
        if (Test-Path $registyPath) {
            $maxInstallDir = (Get-ItemProperty -Path $registyPath -Name InstallDir).InstallDir
            if ($maxInstallDir) {
                return $maxInstallDir
            }
        }
    }
    return $null
}

function normalizePath([string]$path) {
    if ($path.EndsWith("\")) {
        return $path.Substring(0, $path.Length - 1)
    } else {
        return $path;
    }
}

$outputDir = "./.3dsmaxstuffs"
if (-Not (Test-Path $outputDir)) {
    New-Item $outputDir -ItemType Directory
}

$sdkHome = find3dsMaxSDK
if ($null -eq $sdkHome) {
    Write-Error "3DS_MAX_SDK_HOME not found."
    exit 1
}
$sdkHome = normalizePath -Path $sdkHome
$sdkHome | Out-File -FilePath "$outputDir\3DS_MAX_SDK_HOME" -NoNewline

$maxHome = find3dsMax
if ($null -eq $maxHome) {
    Write-Error "3DS_Max_HOME not found."
    exit 1
}
$maxHome = normalizePath -Path $maxHome
$maxHome | Out-File -FilePath "$outputDir\3DS_Max_HOME" -NoNewline