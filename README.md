# AirQualitySensors

[![Build and test](https://github.com/kurta999/AirQualitySensors/actions/workflows/build.yml/badge.svg)](https://github.com/kurta999/AirQualitySensors/actions/workflows/build.yml)

STM32 firmware for an air-quality sensor hub and USB keyboard gateway. It samples five environmental sensors, renders the latest values on an ST7565 LCD, forwards measurements over UART, and can optionally send them through an ESP32 AT-command module over TCP.

The configured target is an **STM32L476ZGTx (Cortex-M4F, LQFP144)**. Pin assignments and generated peripherals live in [AirQualitySensors.ioc](AirQualitySensors.ioc). Measurements are consumed by the companion desktop application, [WindowsAddon](https://github.com/kurta999/WindowsAddon) — see [Backend](#backend).

## Features

- BME680 temperature, humidity, pressure, and raw gas resistance over SPI
- SCD30 CO2, temperature, and humidity over I2C
- VEML6070 raw UV measurement over I2C
- TCS34725 RGB, correlated color temperature, and illuminance over I2C
- Honeywell HPMA115S0 PM2.5 and PM10 parsing over UART
- ST7565 128×64 status display
- USB HID keyboard host with UART forwarding
- Optional ESP32 Wi-Fi/TCP transport, disabled by default
- FreeRTOS task separation and synchronized measurement snapshots
- Debug, Release, and Wi-Fi-check CMake presets, host-side unit tests, and GitHub Actions CI

## Hardware connections

This table reflects the checked-in CubeMX configuration. Confirm voltage levels, pull-ups, sensor addresses, and power requirements against each component's datasheet before connecting hardware.

| Function | Peripheral | MCU pins |
| --- | --- | --- |
| SCD30, VEML6070, TCS34725 | I2C1 | PB8 SCL, PB9 SDA |
| BME680 | SPI2 | PB10 SCK, PC2 MISO, PC3 MOSI, PF12 CS |
| ST7565 LCD | SPI1 | PE13 SCK, PA7 MOSI, PB15 CS, PB13 RESET, PB12 A0, PC9 backlight PWM |
| HPMA115S0 | UART4 | PA0 TX, PA1 RX |
| ESP32 AT module | USART3 + DMA | PC4 TX, PC5 RX, PC1 enable |
| Keyboard gateway UART | USART2 | PA2 TX, PA3 RX |
| USB host | USB OTG FS | PA11 DM, PA12 DP, PG6 power enable |
| Debug | SWD/SWO | PA13 SWDIO, PA14 SWCLK, PB3 SWO |

## Prerequisites

- CMake 3.20 or newer
- Ninja
- GNU Arm Embedded toolchain (`arm-none-eabi-gcc`, `arm-none-eabi-objcopy`, and related tools)
- Optional: STM32CubeIDE/CubeMX for editing the `.ioc` configuration
- Optional: STM32CubeProgrammer or an IDE debugger for flashing

The toolchain file searches `PATH` and common STM32CubeIDE installation locations. For a custom installation, set `ARM_TOOLCHAIN_PATH` to the toolchain's `bin` directory:

```powershell
$env:ARM_TOOLCHAIN_PATH = 'C:\path\to\arm-gnu-toolchain\bin'
```

## Build

Configure and build Debug firmware:

```text
cmake --preset debug
cmake --build --preset debug --parallel
```

For the optimized build, replace `debug` with `release`. Each build produces these files under `build/<preset>/`:

- `AirQualitySensors.elf` — debug symbols and executable image
- `AirQualitySensors.hex` — Intel HEX image
- `AirQualitySensors.bin` — raw flash image
- `AirQualitySensors.map` — linker map

The repository also remains importable in STM32CubeIDE. CMake is the reproducible build used by CI and is the recommended command-line path.

## Backend

Measurements are received by [WindowsAddon](https://github.com/kurta999/WindowsAddon), a Windows tray toolbox that also serves as the sensor backend for this firmware. It runs the TCP server the device reports to, stores the average of the measurements in a configured integration period in an SQLite database, and generates graphs from recent measurements plus daily and weekly averages. The graphs are served at `your_local_ip:2005/graphs` and can be viewed from any device on the network, including a phone.

Point the firmware at it with `APP_SERVER_ADDRESS` and `APP_SERVER_PORT` (see below); the default port `2005` matches its listener. The same measurement record also goes out on USART2, so the backend can be fed over the wired link instead of Wi-Fi.

The record is a shared wire format documented in [docs/protocol.md](docs/protocol.md). The receiver reads its values positionally, so changing the order, count or spacing of the fields breaks the backend even though the firmware still builds. The host test suite pins the exact record to catch that.

## Local configuration and Wi-Fi

The committed defaults in [app_config.h](Core/Inc/app_config.h) disable Wi-Fi and contain no deployment credentials. To enable it:

1. Copy `Core/Inc/app_config.local.example.h` to `Core/Inc/app_config.local.h`.
2. Set the SSID, password, server address, and port in the local file.
3. Reconfigure and rebuild the firmware.

`app_config.local.h` is ignored by Git. Never commit real credentials. Wi-Fi transport is plain TCP, so use it only on a trusted network or put an authenticated, encrypted gateway in front of the device. Remote reset commands remain disabled unless explicitly enabled.

Available configuration macros:

| Macro | Safe default | Purpose |
| --- | --- | --- |
| `APP_WIFI_ENABLED` | `0` | Build and start Wi-Fi tasks |
| `APP_WIFI_SSID` / `APP_WIFI_PASSWORD` | empty | Access-point credentials |
| `APP_SERVER_ADDRESS` | `192.0.2.1` | Measurement receiver address |
| `APP_SERVER_PORT` | `2005` | Measurement receiver port |
| `APP_ALLOW_REMOTE_RESET` | `0` | Accept the ESP32 `RESET` command |
| `APP_MEASUREMENT_SEND_INTERVAL_MS` | `1500` | UART/Wi-Fi reporting interval |
| `APP_SCD30_AMBIENT_PRESSURE_MBAR` | `0` | SCD30 pressure compensation; `0` disables it |

## Tests

The host suite tests the platform-independent protocol and parsers:

```text
cmake -S tests -B build/host-tests -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure
```

GitHub Actions runs these tests and all three firmware presets on every push and pull request. The `wifi-check` preset compiles the ESP32 code path with placeholder credentials, which the Wi-Fi-disabled default builds never reach. Hardware behavior still needs validation on the target board, especially sensor availability, bus timing, display orientation, USB power switching, watchdog timing, and ESP32 firmware compatibility.

## Architecture

```text
sensor tasks / HPMA ISR
          │
          ▼
 synchronized MeasurementSnapshot
      ┌───┴──────────────┐
      ▼                  ▼
 ST7565 UI       bounded text formatter
                         │
                   UART2 / ESP32 TCP
                         │
                         ▼
                  WindowsAddon backend
                   (SQLite + graphs)
```

Application code is under `Core/`, reusable drivers are under `libs/`, generated STM32 and USB support is under `Drivers/`, `Middlewares/`, and `USB_HOST/`, and build logic is under `cmake/`. See [docs/protocol.md](docs/protocol.md) for wire formats.

## Flashing and debugging

Flash `build/debug/AirQualitySensors.elf`, `.hex`, or `.bin` with STM32CubeProgrammer or your normal ST-LINK workflow. The VS Code launch configuration expects the STM32 embedded tools extension and a local ST-LINK GDB server. Do not flash this image to a different MCU without updating the device define, startup file, linker script, CubeMX configuration, and memory layout together.

## Contributing and security

Read [CONTRIBUTING.md](CONTRIBUTING.md) before opening a pull request. Security-sensitive reports belong in the process described by [SECURITY.md](SECURITY.md), not in public issues.

## License

Project-authored code is released under the [MIT License](LICENSE). Bundled vendor and third-party components retain their own terms; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and the license files next to those components.
