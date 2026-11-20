param(
    [switch]$RequireReceiver,
    [string]$ReportPath = (Join-Path (Split-Path $PSScriptRoot -Parent) 'build/nucleo-usb/pc-test.json')
)

$ErrorActionPreference = 'Stop'
$aqsIp = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object IPAddress -eq '192.168.7.2' | Select-Object -First 1
if (-not $aqsIp) { throw 'No DHCP address 192.168.7.2. Connect native USB CN13 with the nucleo-usb firmware running.' }
$aqsAdapter = Get-NetAdapter -InterfaceIndex $aqsIp.InterfaceIndex
if ($aqsIp.PrefixLength -ne 30 -or $aqsIp.PrefixOrigin.ToString() -ne 'Dhcp') {
    throw 'The USB interface did not receive the expected /30 DHCP lease.'
}
$aqsGateway = @(Get-NetRoute -InterfaceIndex $aqsIp.InterfaceIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object DestinationPrefix -eq '0.0.0.0/0')
$aqsDns = @((Get-DnsClientServerAddress -InterfaceIndex $aqsIp.InterfaceIndex -AddressFamily IPv4).ServerAddresses)
if ($aqsGateway.Count -or $aqsDns.Count) { throw 'USB unexpectedly has an IPv4 default gateway or DNS server.' }
$aqsPing = @(Test-Connection -TargetName 192.168.7.1 -Count 3 -ErrorAction Stop)
if (@($aqsPing | Where-Object { $_.Status.ToString() -ne 'Success' }).Count) { throw 'USB ping failed.' }
# Runtime setting changes restart the SCD30 cycle. Allow the documented
# startup/recovery window before judging sensor availability.
$aqsReadyDeadline = (Get-Date).AddSeconds(20)
do {
    $aqsFirst = Invoke-RestMethod 'http://192.168.7.1/api/measurements' -NoProxy -TimeoutSec 5
    if (($aqsFirst.valid_mask -band 0x3B) -eq 0x3B -and
        ($aqsFirst.fresh_mask -band 0x3B) -eq 0x3B) { break }
    Start-Sleep -Milliseconds 500
} while ((Get-Date) -lt $aqsReadyDeadline)
if (($aqsFirst.valid_mask -band 0x3B) -ne 0x3B -or
    ($aqsFirst.fresh_mask -band 0x3B) -ne 0x3B) { throw 'One or more sensor/IAQ sources did not produce fresh data within 20 seconds.' }
$aqsLegacy = (Invoke-WebRequest 'http://192.168.7.1/measurements' -NoProxy -TimeoutSec 5).Content
if ($aqsLegacy -notmatch '^MEAS_DATA SCD30: .+ BME680: .+ HONEYWELL: .+ VEML6070: .+ TCS: ') {
    throw 'The legacy measurement record is missing or malformed.'
}
# Exercise connection cleanup and memory reclamation as well as a single GET.
for ($aqsAttempt = 0; $aqsAttempt -lt 20; ++$aqsAttempt) {
    $aqsSample = Invoke-RestMethod 'http://192.168.7.1/api/measurements' -NoProxy -TimeoutSec 5
    if ($aqsSample.valid_mask -ne $aqsFirst.valid_mask) { throw 'Sensor validity changed during HTTP requests.' }
}
Start-Sleep -Seconds 4
$aqsSecond = Invoke-RestMethod 'http://192.168.7.1/api/measurements' -NoProxy -TimeoutSec 5
if ($aqsSecond.scd30.timestamp_ms -le $aqsFirst.scd30.timestamp_ms -or
    $aqsSecond.bme680.timestamp_ms -le $aqsFirst.bme680.timestamp_ms -or
    $aqsSecond.iaq.timestamp_ms -le $aqsFirst.iaq.timestamp_ms) {
    throw 'Measurements did not advance across the USB link.'
}
$aqsReceiverWorking = $aqsSecond.usb.acknowledged_bytes -gt $aqsFirst.usb.acknowledged_bytes
if ($RequireReceiver -and -not $aqsReceiverWorking) { throw 'The PC measurement receiver did not acknowledge new TCP data.' }
$aqsReport = [ordered]@{
    checked_at = (Get-Date).ToString('o')
    adapter = $aqsAdapter.InterfaceDescription
    interface = $aqsAdapter.Name
    address = $aqsIp.IPAddress
    prefix_length = $aqsIp.PrefixLength
    address_source = $aqsIp.PrefixOrigin.ToString()
    usb_default_gateway = $false
    usb_dns_server = $false
    ping_successes = $aqsPing.Count
    http_requests_passed = 23
    sensor_valid_mask = ('0x{0:X}' -f $aqsSecond.valid_mask)
    measurements_advanced = $true
    tcp_receiver_acknowledged_new_data = $aqsReceiverWorking
    records_sent = $aqsSecond.usb.records_sent
    acknowledged_bytes = $aqsSecond.usb.acknowledged_bytes
    measurements = $aqsSecond
}
New-Item -ItemType Directory -Path (Split-Path $ReportPath -Parent) -Force > $null
$aqsReport | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $ReportPath -Encoding utf8
([pscustomobject]$aqsReport) | Select-Object adapter,interface,address,address_source,ping_successes,http_requests_passed,
    sensor_valid_mask,measurements_advanced,tcp_receiver_acknowledged_new_data | Format-List
Write-Host "Saved USB network test report: $ReportPath"
