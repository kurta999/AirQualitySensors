# AirQualitySensors

[![Build and test](https://github.com/kurta999/AirQualitySensors/actions/workflows/build.yml/badge.svg)](https://github.com/kurta999/AirQualitySensors/actions/workflows/build.yml)

STM32 firmware for an air-quality sensor hub and USB keyboard gateway. It samples five environmental sensors, renders the latest values on an ST7565 LCD, forwards measurements over UART, and can optionally send them through an ESP32 AT-command module over TCP.

The configured target is an **STM32L476ZGTx (Cortex-M4F, LQFP144)**. Pin assignments and generated peripherals live in [AirQualitySensors.ioc](AirQualitySensors.ioc); the full pin map and per-peripheral configuration are documented in [Pinout and hardware connections](#pinout-and-hardware-connections) at the end of this file. Measurements are consumed by the companion desktop application, [WindowsAddon](https://github.com/kurta999/WindowsAddon) — see [Backend](#backend).

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

## Pinout and hardware connections

Everything below is taken from the checked-in configuration: pin assignments come from [AirQualitySensors.ioc](AirQualitySensors.ioc) and the generated initialisation code in `Core/Src/` (`gpio.c`, `i2c.c`, `spi.c`, `usart.c`, `tim.c`, `adc.c`, `iwdg.c`, `dma.c`), and the pin labels come from the `*_Pin` / `*_GPIO_Port` macros in [Core/Inc/main.h](Core/Inc/main.h). Sensor addresses, bus speeds and CS handling come from the drivers that use them.

Target: **STM32L476ZGTx**, Cortex-M4F, LQFP144, 42 pins configured. Confirm voltage levels, pull-ups, sensor addresses and power requirements against each component's datasheet before connecting hardware.

### Quick reference

| Function | Peripheral | MCU pins |
| --- | --- | --- |
| SCD30, VEML6070, TCS34725 | I2C1 | PB8 SCL, PB9 SDA |
| BME680 | SPI2 | PB10 SCK, PC2 MISO, PC3 MOSI, PF12 CS |
| ST7565 LCD | SPI1 (half-duplex) | PE13 SCK, PA7 MOSI, PB15 CS, PB13 RESET, PB12 A0, PC9 backlight PWM |
| HPMA115S0 | UART4 | PA0 TX, PA1 RX |
| ESP32 AT module | USART3 + DMA | PC4 TX, PC5 RX, PC1 enable |
| Keyboard / measurement gateway UART | USART2 | PA2 TX, PA3 RX |
| USB HID host | USB OTG FS | PA11 DM, PA12 DP, PG6 VBUS enable |
| Spare I2C | I2C3 | PC0 SCL, PG8 SDA |
| Analog input | ADC1_IN16 | PB1 |
| Indicator LEDs | GPIO | PC7, PB7, PB14, PD4–PD7 |
| Debug | SWD/SWO | PA13 SWDIO, PA14 SWCLK, PB3 SWO |

### Complete pin map

| Pin | Signal | AF | Mode | Label in code | Used for |
| --- | --- | --- | --- | --- | --- |
| PA0 | UART4_TX | AF8 | Push-pull AF, very high speed | — | HPMA115S0 particle sensor command line |
| PA1 | UART4_RX | AF8 | Push-pull AF, very high speed | — | HPMA115S0 measurement frames |
| PA2 | USART2_TX | AF7 | Push-pull AF, very high speed | `UART_KBD_TX` | Keystroke + measurement records to the host |
| PA3 | USART2_RX | AF7 | Push-pull AF, very high speed | `UART_KBD_RX` | Host link (receive side) |
| PA7 | SPI1_MOSI | AF5 | Push-pull AF, very high speed | — | ST7565 LCD data (half-duplex, no MISO) |
| PA11 | USB_OTG_FS_DM | AF10 | Push-pull AF | — | USB HID host D− |
| PA12 | USB_OTG_FS_DP | AF10 | Push-pull AF | — | USB HID host D+ |
| PA13 | SYS_JTMS-SWDIO | — | Trace async SW | — | SWD debug |
| PA14 | SYS_JTCK-SWCLK | — | Trace async SW | — | SWD debug |
| PB1 | ADC1_IN16 | — | `GPIO_MODE_ANALOG_ADC_CONTROL` | — | Analog input, rank 1 of the ADC sequence (see [ADC1](#adc1-pb1)) |
| PB3 | SYS_JTDO-SWO | — | Trace async SW | — | SWO trace; `printf`/`ITM_SendChar` output |
| PB7 | GPIO output | — | Push-pull, no pull, low speed | `ULED_2` | Startup blink in `main()` |
| PB8 | I2C1_SCL | AF4 | Open-drain AF, very high speed, **no internal pull** | — | Sensor I2C clock |
| PB9 | I2C1_SDA | AF4 | Open-drain AF, very high speed, **no internal pull** | — | Sensor I2C data |
| PB10 | SPI2_SCK | AF5 | Push-pull AF, very high speed | — | BME680 clock |
| PB12 | GPIO output | — | Push-pull, no pull, low speed | `LCD_A0` | ST7565 data/command select |
| PB13 | GPIO output | — | Push-pull, no pull, low speed | `LCD_RES` | ST7565 reset |
| PB14 | GPIO output | — | Push-pull, no pull, low speed | `ULED_3` | Wi-Fi send heartbeat and `Error_Handler()` blink |
| PB15 | GPIO output | — | Push-pull, no pull, low speed | `LCD_CS` | ST7565 chip select (software driven) |
| PC0 | I2C3_SCL | AF4 | Open-drain AF, very high speed | — | Spare I2C bus (initialised, no driver) |
| PC1 | GPIO output | — | Push-pull, no pull, low speed | `WIFI_EN` | ESP32 enable line |
| PC2 | SPI2_MISO | AF5 | Push-pull AF, very high speed | — | BME680 data out |
| PC3 | SPI2_MOSI | AF5 | Push-pull AF, very high speed | — | BME680 data in |
| PC4 | USART3_TX | AF7 | Push-pull AF, very high speed | — | ESP32 AT commands |
| PC5 | USART3_RX | AF7 | Push-pull AF, very high speed | — | ESP32 AT responses (DMA) |
| PC7 | GPIO output | — | Push-pull, no pull, low speed | `ULED_1` | Reserved indicator, driven low at init |
| PC9 | TIM3_CH4 | AF2 | Push-pull AF, low speed | `LCD_PWN` | LCD backlight PWM |
| PD4 | GPIO output | — | Push-pull, no pull, low speed | `LED_1` | Reserved indicator, driven low at init |
| PD5 | GPIO output | — | Push-pull, no pull, low speed | `LED_2` | Reserved indicator, driven low at init |
| PD6 | GPIO output | — | Push-pull, no pull, low speed | `LED_3` | Reserved indicator, driven low at init |
| PD7 | GPIO output | — | Push-pull, no pull, low speed | `LED_4` | Reserved indicator, driven low at init |
| PE13 | SPI1_SCK | AF5 | Push-pull AF, very high speed | — | ST7565 LCD clock |
| PF12 | GPIO output | — | Push-pull, no pull, low speed | `SPI2_CS` | BME680 chip select (software driven) |
| PG6 | GPIO output | — | Push-pull, no pull, low speed | `USB_EN` | USB VBUS charge-pump enable |
| PG8 | I2C3_SDA | AF4 | Open-drain AF, very high speed | — | Spare I2C bus (initialised, no driver) |

Port G is powered from VDDIO2, so `MX_GPIO_Init()` calls `HAL_PWREx_EnableVddIO2()` before configuring PG6 and PG8. Without that call both pins stay dead.

### I2C1 — sensor bus (PB8 SCL, PB9 SDA)

- Timing word `0x00909BEB`, 7-bit addressing, analog filter enabled, digital filter 0, no clock stretching disable.
- Pins are open-drain with `GPIO_NOPULL`, so **external pull-up resistors are required** (typically 4.7 kΩ to the sensor supply).
- Three devices share the bus:

| Device | 7-bit address | Driver | Notes |
| --- | --- | --- | --- |
| SCD30 (CO2 / temperature / humidity) | `0x61` | `Core/Src/scd_sensor.c` + `libs/scd30` | 2 s measurement interval, polled non-blocking via `scd30_get_data_ready()` |
| VEML6070 (UV) | `0x38` low byte, `0x39` high byte, `0x0C` ARA | `Core/Src/uv_sensor.c` | Integration time 4T; ARA read on init clears a pending alert |
| TCS34725 (RGB / lux / colour temperature) | `0x29` | `Core/Src/light_sensor.c` | 50 ms integration, 4× gain, ID register accepts `0x4D`, `0x44`, `0x10` |

Because the SCD30 and VEML6070 are serviced by `SensorsTask` while the TCS34725 is serviced by the lower-priority `SensorsBme680Task`, all bus access goes through the mutex in [`Core/Src/i2c_bus.c`](Core/Src/i2c_bus.c) (`I2cBus_Lock()` / `I2cBus_Unlock()`, 1000 ms timeout). The lock is taken around a whole sensor operation, never around individual HAL calls, and it is not recursive.

### I2C3 — spare bus (PC0 SCL, PG8 SDA)

Configured identically to I2C1 and initialised from `main()` (`MX_I2C3_Init()`), but no driver in the tree uses `hi2c3`. Treat it as a free expansion bus; it also needs external pull-ups.

### SPI1 — ST7565 LCD (PE13 SCK, PA7 MOSI)

- Master, **half-duplex (`SPI_DIRECTION_1LINE`)** — the display has no MISO connection.
- CPOL low, CPHA 1-edge (SPI mode 0), MSB first, 8-bit, baud prescaler 2, software NSS.
- `SPI1_IRQn` enabled at preemption priority 5.
- Control pins driven by the driver, not by the SPI peripheral:

| Pin | Label | Role |
| --- | --- | --- |
| PB15 | `LCD_CS` | Chip select |
| PB13 | `LCD_RES` | Hardware reset |
| PB12 | `LCD_A0` | Data / command select |
| PC9 | `LCD_PWN` | Backlight, TIM3_CH4 PWM |

`UI_Init()` in [`Core/Src/user_interface.c`](Core/Src/user_interface.c) passes these to `ST7565_Init()` together with contrast `35` and direction `0` (not flipped), then starts TIM3 CH4 and sets the compare value to `255` (full backlight). Display access is serialised with `lcd_mutexHandle`.

### SPI2 — BME680 (PB10 SCK, PC2 MISO, PC3 MOSI, PF12 CS)

- Master, full-duplex, CPOL low / CPHA 1-edge (SPI mode 0), MSB first, 8-bit, baud prescaler 8, software NSS with `NSSPMode = SPI_NSS_PULSE_ENABLE`.
- **CS is a plain GPIO**: `SensorsBme680Task()` drives `SPI2_CS_Pin` (PF12) high at task start, then low around each register read/write in `sensor_spi_read()` / `sensor_spi_write()`, with 350 ms HAL timeouts.
- Sensor settings: humidity ×2, pressure ×4, temperature ×8 oversampling, IIR filter size 3, gas heater 320 °C for 150 ms, forced mode. The loop period follows `bme680_get_profile_dur()`, clamped to a 50 ms floor and falling back to 1000 ms while the sensor has not initialised.

### UART4 — Honeywell HPMA115S0 (PA0 TX, PA1 RX)

- **9600 baud**, 8N1, no flow control, 16× oversampling.
- `UART4_IRQn` at preemption priority 5; reception is interrupt-driven one byte at a time (`HAL_UART_Receive_IT`) and fed into the frame parser in `libs/hpma/hpma_parser.c`.
- `Honeywell_Init()` sends `68 01 40 57` (enable auto-send) and `68 01 01 96` (start measurement).
- `Honeywell_Poll()` rearms reception after HAL aborts it on overrun/noise/framing errors, and fully reinitialises the sensor after `HONEYWELL_SILENCE_TIMEOUT_MS` (10 s) without a decoded frame. A missing particle sensor never blocks the rest of the system.

### USART2 — host / keyboard gateway (PA2 TX, PA3 RX)

- **921600 baud**, 8N1, no flow control. `USART2_IRQn` at preemption priority 5.
- Carries two things from `StartDefaultTask()`: USB HID keyboard reports (CRC-16 appended, see `Protocol_Crc16`) and the periodic measurement record described in [docs/protocol.md](docs/protocol.md), sent every `APP_MEASUREMENT_SEND_INTERVAL_MS`.

### USART3 — ESP32 AT module (PC4 TX, PC5 RX, PC1 enable)

- Configured with the **LL** driver, not HAL: 115200 baud, 8N1, one-bit sampling enabled, overrun detection disabled, DMA-deactivate-on-RX-error disabled.
- Clock source PCLK1. `USART3_IRQn` at preemption priority 5.
- DMA (see the [DMA map](#dma-and-interrupt-map)): RX on DMA1 channel 3, circular, byte-aligned; TX on DMA1 channel 2, normal mode. RX circular buffer is 4096 bytes (`ESP32_RX_CIRCULAT_BUF_SIZE`).
- `Wifi_Init()` passes `WIFI_EN_GPIO_Port` / `WIFI_EN_Pin` (PC1) as the enable line and **NULL for the reset line** — no ESP32 reset pin is wired. `wifi_setup()` drives PC1 low before issuing a software reset.
- The whole Wi-Fi path, including these pins, is compiled out unless `APP_WIFI_ENABLED` is `1`.

### USB OTG FS host (PA11 DM, PA12 DP, PG6 VBUS enable)

- PA11/PA12 on AF10, `OTG_FS_IRQn` at preemption priority 5, HID host class.
- `MX_DriverVbusFS()` in [`USB_HOST/Target/usbh_platform.c`](USB_HOST/Target/usbh_platform.c) switches VBUS through PG6: state `1` sets the pin, state `0` clears it. Match this against the polarity of your charge pump before powering a device.

### ADC1 (PB1)

Three-channel scan, 12-bit, right-aligned, continuous, with 256× oversampling and an 8-bit right shift, sampling time 640.5 cycles per channel, clocked from PLLSAI1:

| Rank | Channel | Source |
| --- | --- | --- |
| 1 | `ADC_CHANNEL_16` | PB1 |
| 2 | `ADC_CHANNEL_TEMPSENSOR` | Internal die temperature |
| 3 | `ADC_CHANNEL_VREFINT` | Internal reference |

Results land in `adc_dma_buf[3]` over DMA1 channel 1 in circular mode. Note that `StartDmaConversion()` is defined in `Core/Src/adc.c` but is **not called anywhere**, so the ADC is initialised and idle — PB1 is reserved rather than actively sampled.

### Timers

| Timer | Configuration | Purpose |
| --- | --- | --- |
| TIM3 CH4 | Prescaler 1, period 255, PWM on PC9 (AF2) | LCD backlight; duty set to 255 in `UI_Init()` |
| TIM16 | Prescaler 40000, period 9 → 10 ms tick, `TIM1_UP_TIM16_IRQn` priority 5 | Drives `Wifi_10MsIsr()`; started only when Wi-Fi is enabled |
| TIM17 | HAL time base (`SYS_VS_tim17`) | `HAL_IncTick()` and `Wifi_SysTick()` |

### Watchdog

IWDG with prescaler 32 and reload/window 3999 — roughly a **4 s** timeout on the ~32 kHz LSI. It is refreshed from `StartDefaultTask()` only while `TaskHealth_AllAlive()` is true, so a wedged task resets the board instead of being masked. In `DEBUG` builds the counter is frozen on halt (`__HAL_DBGMCU_FREEZE_IWDG()`).

### LEDs and indicators

All are push-pull outputs with no pull, low speed, and are driven low in `MX_GPIO_Init()`.

| Pin | Label | Behaviour in the current firmware |
| --- | --- | --- |
| PC7 | `ULED_1` | Configured only; never driven after init |
| PB7 | `ULED_2` | Blinks 10× during startup in `main()`, before the scheduler starts |
| PB14 | `ULED_3` | Toggled on every successful Wi-Fi measurement send; also toggled at 200 ms in `Error_Handler()` |
| PD4–PD7 | `LED_1`–`LED_4` | Configured only; never driven after init — free for board bring-up |

### DMA and interrupt map

| DMA channel | Request | Direction | Mode | Owner |
| --- | --- | --- | --- | --- |
| DMA1 channel 1 | 0 | Peripheral → memory, half-word | Circular | ADC1 |
| DMA1 channel 2 | 2 | Memory → peripheral, byte | Normal | USART3 TX (ESP32) |
| DMA1 channel 3 | 2 | Peripheral → memory, byte | Circular | USART3 RX (ESP32) |

Every application interrupt — `DMA1_Channel1/2/3`, `TIM1_UP_TIM16`, `TIM1_TRG_COM_TIM17`, `SPI1`, `USART2`, `USART3`, `UART4`, `OTG_FS` — is set to preemption priority 5, subpriority 0. `PendSV` is set to 15 so FreeRTOS context switches stay the lowest priority.

### Wiring notes

- I2C1 and I2C3 have no internal pull-ups configured; add external ones.
- PG6 (`USB_EN`) and PG8 (`I2C3_SDA`) require VDDIO2 to be supplied and enabled.
- SPI1 runs half-duplex — do not expect to read back from the ST7565.
- Both chip selects (PB15 for the LCD, PF12 for the BME680) are software-controlled GPIOs, not hardware NSS.
- The board is clocked from MSI range 6 through the PLL (PLLM 1, PLLN 20, PLLR /2), with PLLSAI1 feeding the ADC and USB; there is no external crystal in the configuration.
- Changing any assignment here means changing the `.ioc` **and** the generated init code together, or the next CubeMX regeneration will silently revert it.
