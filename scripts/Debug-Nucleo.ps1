param(
    [ValidateSet('nucleo-i2c', 'nucleo-sensors', 'nucleo-bsec')]
    [string]$Preset = 'nucleo-i2c',
    [switch]$AttachOnly,
    [switch]$NoBuild,
    [int]$Port = 3333
)

$ErrorActionPreference = 'Stop'
$aqsRoot = Split-Path $PSScriptRoot -Parent

function Find-AqsTool([string]$Name, [string]$Plugin) {
    $command = Get-Command $Name -ErrorAction SilentlyContinue
    if ($command) { return $command.Source }
    $pattern = "C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/$Plugin/tools/bin/$Name.exe"
    $tool = Get-ChildItem $pattern | Sort-Object FullName -Descending | Select-Object -First 1
    if (-not $tool) { throw "Cannot find $Name. Install STM32CubeIDE or add it to PATH." }
    return $tool.FullName
}

Push-Location $aqsRoot
try {
    $aqsGdb = Find-AqsTool 'arm-none-eabi-gdb' 'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*'
    $aqsServer = Find-AqsTool 'ST-LINK_gdbserver' 'com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.*'
    $aqsProgrammer = Find-AqsTool 'STM32_Programmer_CLI' 'com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*'
    if (-not $NoBuild -and -not $AttachOnly) {
        $aqsCmake = Find-AqsTool 'cmake' 'com.st.stm32cube.ide.mcu.externaltools.cmake.*'
        $aqsNinja = Find-AqsTool 'ninja' 'com.st.stm32cube.ide.mcu.externaltools.ninja.*'
        $aqsOriginalPath = $env:PATH
        try {
            $env:PATH = "$(Split-Path $aqsNinja);$env:PATH"
            & $aqsCmake --preset $Preset
            if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed.' }
            & $aqsCmake --build --preset $Preset --parallel
            if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed.' }
        } finally { $env:PATH = $aqsOriginalPath }
    }
    $aqsBuildDir = "build/$Preset"
    $aqsElf = "$aqsBuildDir/AirQualitySensors.elf"
    if (-not (Test-Path $aqsElf)) { throw "Missing $aqsElf; build the $Preset preset first." }
    $aqsConnection = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
    $aqsServerProcess = $null
    if (-not $aqsConnection) {
        $aqsProgrammerDir = Split-Path $aqsProgrammer
        $aqsMode = if ($AttachOnly) { '-g' } else { '-k' }
        $aqsServerProcess = Start-Process -FilePath $aqsServer -WindowStyle Hidden -PassThru `
            -ArgumentList @('-d', '-e', $aqsMode, '-p', $Port, '--frequency', '1000', '-cp', "`"$aqsProgrammerDir`"") `
            -RedirectStandardOutput "$aqsBuildDir/stlink-server.log" `
            -RedirectStandardError "$aqsBuildDir/stlink-server-error.log"
        $aqsDeadline = [DateTime]::UtcNow.AddSeconds(15)
        do {
            if ($aqsServerProcess.HasExited) { throw 'ST-LINK server exited; inspect stlink-server.log.' }
            Start-Sleep -Milliseconds 200
            $aqsConnection = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
        } while (-not $aqsConnection -and [DateTime]::UtcNow -lt $aqsDeadline)
        if (-not $aqsConnection) { throw 'ST-LINK server did not open its debug port.' }
    }
    $aqsStartup = @(
        'set pagination off', 'set confirm off',
        "target extended-remote localhost:$Port"
    )
    if (-not $AttachOnly) { $aqsStartup += @('load', 'monitor reset', 'monitor halt') }
    $aqsStartup += @(
        'break Error_Handler', 'break HardFault_Handler',
        'define sensors',
        'print measurements',
        'print scd30_diagnostics',
        'print hi2c1.ErrorCode',
        'print/x *(unsigned int*)0x48000410'
    )
    if ($Preset -ne 'nucleo-i2c') { $aqsStartup += 'print bme680_diagnostics' }
    if ($Preset -eq 'nucleo-bsec') { $aqsStartup += 'print bsec_diagnostics' }
    $aqsStartup += 'end'
    if (-not $AttachOnly) { $aqsStartup += @('tbreak main', 'continue') }
    $aqsStartup | Set-Content "$aqsBuildDir/debug.gdb" -Encoding ascii
    Write-Host 'GDB: continue to run; Ctrl+C to pause; sensors to print readings; quit to leave.'
    try {
        & $aqsGdb -q $aqsElf -x "$aqsBuildDir/debug.gdb"
    } finally {
        if ($aqsServerProcess -and -not $aqsServerProcess.HasExited) {
            Stop-Process -Id $aqsServerProcess.Id
        }
    }
} finally { Pop-Location }
