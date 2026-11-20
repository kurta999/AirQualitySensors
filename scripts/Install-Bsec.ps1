# Downloads the selected official binary locally; the package stays Git-ignored.
$ErrorActionPreference = 'Stop'
$aqsRoot = Split-Path $PSScriptRoot -Parent
$aqsRevision = '4f559a6aa450f0ce436e602b42dc384f52128c66'
$aqsPackage = Join-Path $aqsRoot 'libs/bsec'
$aqsFiles = @{
    'src/cortex-m4/fpv4-sp-d16-hard/libalgobsec.a' = 'AD831FCB676E7CAFD491542B2277B773113CE5F92EC022945BEBB53DD8595650'
    'src/inc/bsec_interface.h' = '7643D4DBC0958C9403D5477EB73F9453E54CF69D3BA99CAB1C877A0CD4570AE1'
    'src/inc/bsec_datatypes.h' = 'A632C78CA4E38AD7773FE3493ECB2FF2054A0A5EEC429C091DB9BFE7ED55F927'
    'src/inc/bsec_interface_multi.h' = 'FC0DC1ADE47BC7659F8291BB7BB5A74A689876ADF8F738D516FE54C1C492DC7A'
    'src/config/bme680/bme680_iaq_33v_3s_4d/bsec_iaq.txt' = 'ADF1509B0D07D2EEC90D46AC698205DE32D899855D95FA054BD2A5464910DE6A'
    'LICENSE.md' = 'A27FD42924746A4B45808C21DC71F7C0E6006B4CE9F933D2301E154ED4255D76'
    'library.properties' = '269CC7A2061FFD1136F474C5533B84747A6128943E5FF9562F0CBCA7492E6861'
}

foreach ($aqsPath in $aqsFiles.Keys) {
    $aqsTarget = Join-Path $aqsPackage $aqsPath
    if ((Test-Path -LiteralPath $aqsTarget) -and
        (Get-FileHash -LiteralPath $aqsTarget -Algorithm SHA256).Hash -eq $aqsFiles[$aqsPath]) {
        continue
    }
    New-Item -ItemType Directory -Path (Split-Path $aqsTarget) -Force > $null
    $aqsDownload = "$aqsTarget.download"
    $aqsUrl = "https://raw.githubusercontent.com/boschsensortec/Bosch-BSEC2-Library/$aqsRevision/$aqsPath"
    Invoke-WebRequest -Uri $aqsUrl -OutFile $aqsDownload
    if ((Get-FileHash -LiteralPath $aqsDownload -Algorithm SHA256).Hash -ne $aqsFiles[$aqsPath]) {
        throw "Checksum mismatch for $aqsPath. The downloaded file was not installed."
    }
    Move-Item -LiteralPath $aqsDownload -Destination $aqsTarget -Force
}
Write-Host "Installed Bosch BSEC at $aqsRevision for Cortex-M4F (GCC hard float)."
Write-Host "Bosch binary license and example notices: $aqsPackage/LICENSE.md"
