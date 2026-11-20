# Embedded dashboard

Use `nucleo-usb` on the NUCLEO-L496ZG. Both `http://192.168.7.1/` and
`http://air.local/` reach the device over CN13. mDNS advertises an A record and
`_http._tcp` service with 120-second TTL. The hostname and station label are
editable. IP/subnet remain 192.168.7.1/30, DHCP client 192.168.7.2/30, with no
gateway/DNS. Older presets retain their original transport and build controls.

## Frontend build

In `web/`, run `npm ci`, `npm run build`, and `npm test` (Node 22 or newer).
TypeScript is checked before esbuild bundles the selected Chart.js line/scale/
tooltip/fill components. No framework, font download, CDN, or date library is
needed. The browser draws each measurement and BSEC status independently.
Intervals/categories, CSV, missing/stale data, calibration status, offline
recovery, and explicit save/error feedback are included. No synthetic history
is displayed.

The build writes ignored preview files into `web/dist/`, plus the **tracked**
`Core/Src/web_assets.c`. Commit that C file alongside frontend changes so
offline/CI firmware builds contain the same page. The generated `const` arrays
contain gzip only. HTTP sends `Content-Encoding: gzip`, proper content types,
and a bounded stream directly from flash. The browser decompresses the assets.
The combined gzip budget is enforced at 110 KiB; the initial build is about
70 KiB. `web/dist/manifest.json` records exact sizes and SHA-256 hashes.

## Settings

`GET /api/config` returns `config`, `capabilities`, `storage`, and `network`.
All fields in `config` are runtime settings and appear on Configuration.
`POST /api/config` accepts a bounded URL-encoded patch. Fields omitted from a
patch retain their values. The frontend sends the full form. Unknown/duplicate
fields, malformed escapes/numbers, invalid names/IPs, and out-of-range values
are rejected atomically with HTTP 400. Errors do not change live settings.
Host must match the device IP or configured mDNS name. Writes also require
`Content-Type: application/x-www-form-urlencoded` and `X-Air-Config: 1`; when
Origin is supplied it must match Host. Cross-origin preflights are rejected.
There is no login; access is through the directly attached USB network.

| Setting | Range / meaning |
| --- | --- |
| `master_enabled`, `receiver_enabled`, `scd_asc`, `scd_recovery` | 0 or 1 |
| `sensor_mask` | SCD30=1, BME680/BSEC=2, UV=8, color=16 |
| `sensor_poll_ms` | 250–5000 ms |
| `light_poll_ms` | 100–5000 ms |
| `send_interval_ms` | 250–3600000 ms |
| `receiver_port`, `receiver_ip` | 1–65535, unicast IPv4 |
| `history_interval_s` | 0 (automatic 150 s), or 5–3600 s |
| `scd_interval_s` | 2–1800 s |
| `scd_pressure_mbar` | 0, or 700–1400 mbar; overrides altitude |
| `scd_temperature_offset` | 0–2000 hundredths °C |
| `scd_altitude_m` | 0–3000 m |
| `bsec_heat_offset` | −2000–2000 hundredths °C |
| `bsec_save_interval_s` | 3600–86400 s; accuracy improvements also save |
| `light_gain`, `light_cycles` | 0/1/2/3 = 1×/4×/16×/60×; 1–256 cycles × 2.4 ms |
| `uv_integration` | 0/1/2/3 = ½T/1T/2T/4T |
| `hostname`, `device_name` | 1–31 chars; lowercase DNS label / simple station label |

Sensor disable stops conversions/heater scheduling and invalidates readings;
the master switch pauses all attached sensors while USB/configuration stays
available. SCD30 changes restart its measurement cycle and read back settings;
nonvolatile sensor offsets/ASC are rewritten only when changed. BSEC owns
its heater/oversampling/sample schedule. Pin mapping, peripheral inclusion,
USB identity, clocks, and transport selection remain build-time capabilities.

Two alternating 4-KiB settings slots at **0x080FC000/0x080FD000** use the tested
record codec (version/tag, sequence, CRC32, commit written last). Settings are
verified before publishing to tasks. Interrupted saves preserve the old slot.
Unchanged settings do not erase flash. BSEC slots at **0x080FE000/0x080FF000**
remain independent. The linker reserves all 16 KiB and requires code/constants
in bank 1 while bank 2 is programmed. Flash layout is checked at runtime before
reading/erasing. Full-chip erase clears saved settings and calibration.

## History and memory

600 × **96-byte** records = **57,600 bytes**, in the dedicated SRAM2 NOLOAD
section (`0x10000000`, 64 KiB). Count/indices reset on boot; uninitialized RAM2
is never read. Full-day retention selects 150 s per point, rounded to fit the
fixed buffer with room for endpoints. Data older than 24 hours is pruned;
faster sampling is a normal ring overwrite. Interval changes empty the ring.
Time comparisons handle 32-bit tick wrap. All append/export operations run in
the same network task, and each HTTP history page is an immutable small copy.
The latest measurement is overlaid in the browser between recorded points.

Main SRAM is separate from history and includes the 32-KiB RTOS heap, task
stacks, lwIP heap, packet pools, four fixed HTTP connection slots, and API
buffers. `/api/config` reports static main-RAM reservation, remaining RTOS heap,
firmware image size, and history reservation. The linker enforces bank sizes;
the gzip file is never copied whole to RAM. No allocation grows with uptime.

## HTTP API

`GET /api/measurements` keeps the original nested snapshot schema and adds:
`values[21]`, `fresh_mask`, `config_revision`, and `history_count`. Validity
indicates a source has produced data; freshness also accounts for sensor
disable, elapsed time, and the configured SCD30 measurement interval.
`GET /measurements` and raw `SEND_SENSOR_DATA` preserve the companion protocol.

`GET /api/history?after=<sequence>` returns at most 32 binary records, in
chronological order after that cursor. Use 0 to start with the oldest. Repeat
with the last returned sequence until fewer than 32 records arrive. Sequence
numbers continue across interval changes, and restart after device reset.
The browser detects a reboot through uptime and refetches history/configuration.

All wire fields are little-endian. The 32-byte header is eight uint32 values:
magic **0x31524941** (`AIR1`), version=1, record size=96, columns=21, record count,
current device tick (ms), sampling interval (seconds), first returned sequence
(0 when empty). A record contains tick uint32, valid_mask uint32, 21 float32
values, then four bytes: BSEC accuracy, stabilized, run-in, reserved=0.

| Column | Value |
| --- | --- |
| 0–2 | SCD30 CO₂ ppm, temperature °C, RH % |
| 3–6 | BME680 temperature °C, RH %, pressure hPa, gas Ω |
| 7 | VEML6070 raw counts |
| 8–10 | TCS34725 RGB, clear-normalized ×255 |
| 11–12 | Color temperature K, lux |
| 13–18 | BSEC IAQ, static IAQ, CO₂-equivalent ppm, bVOC-equivalent ppm, compensated °C, compensated RH % |
| 19–20 | HPMA PM2.5/PM10 µg/m³ (invalid/unavailable on this USB profile) |

Per-source validity bits match `MeasurementSource`. Accuracy/status use the IAQ
bit. Missing/stale data is graphed as gaps, never invented zero readings.
The UV counts are not a calibrated UV index; estimated BSEC eCO₂ is separate
from the SCD30 CO₂ measurement. IAQ accuracy remains visible while learning.

HTTP framing is bounded: 1536-byte headers, 1536-byte body, 3071-byte complete
request, four connections, no chunked requests/pipelining, 10-second inactivity
and 60-second total timeouts. TCP ACK callbacks stream flash assets in MSS-sized
chunks. A buffer slot cannot be reused until its data is acknowledged or the
TCP PCB is aborted, preventing retransmission from referencing overwritten RAM.

## Verification

Host C tests cover atomic config parsing, invalid/malicious values, fragmented
HTTP framing, duplicate headers, write-origin restrictions, ring overwrite,
pagination, tick wrap, and 24-hour retention. Existing record/CRC/interrupted-
flash-write, sensor retry, and legacy protocol tests remain in the suite.
`npm test` checks browser decoding, field layout, partial records, validity,
nonfinite data, and wrap handling.

With the real board running, `node web/test-device.mjs` uses installed Chrome
to verify live sensors, all 22 graphs, filters/range, flash-save feedback,
master disable/resume, mobile overflow, CSV download, and no external requests
or browser errors. It temporarily sets 5-second history/2-second transmission.
Reset the MCU and check those fields/flash sequence persist, then run
`node web/test-device.mjs --restore` to restore the saved original intervals.
Screenshots/reports are in ignored `build/nucleo-usb/`. The separate
`scripts/Test-UsbNetwork.ps1 -RequireReceiver` checks DHCP/ping/legacy receiver
delivery. The 24-hour retention is exercised by host tests; live hardware
testing need not wait a whole day.

`node web/test-http.mjs` checks gzip byte hashes against the local build,
air.local lookup, write restrictions, temporary hostname/sensor setting changes,
four simultaneous transfers, and real history pages. Run it after the browser
probe has collected several 5-second samples; it restores the settings it changes.
Set `AIR_DEVICE_URL=http://air.local` for the browser runner to test the mDNS URL
directly. Final firmware should be left in automatic history mode.
