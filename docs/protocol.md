# Serial and TCP protocol

The firmware emits two independent kinds of data on USART2 and optionally sends measurement text through the ESP32 TCP connection.

## Measurement record

Measurements are UTF-8/ASCII text terminated by `\n`:

```text
MEAS_DATA SCD30: <temperature>,<humidity>,<co2> BME680: <temperature>,<humidity>,<pressure>,<gas_resistance>,<timestamp_ms> HONEYWELL: <pm25>,<pm10> VEML6070: <uv_raw> TCS: <red>,<green>,<blue>,<color_temperature>,<lux>\n
```

Floating-point values use four decimal places. Integers are unsigned decimal values. The same record format is used for USART2 and ESP32 TCP output. Measurement records do not carry a CRC or length prefix; receivers should read through the newline and reject incomplete or oversized lines.

This record is a contract with the backend, [WindowsAddon](https://github.com/kurta999/WindowsAddon), which reads the values positionally. The field order and count must not change without updating the receiver in the same breath; adding, removing or reordering a field breaks it even though the firmware still compiles. The host test suite pins the exact record so an accidental change fails there first. Per-sensor validity and last-update timestamps are tracked inside the firmware and shown on the LCD, but are deliberately kept off the wire for this reason.

## Keyboard gateway messages

At USB-host startup, USART2 sends the five ASCII bytes `reset`. While a HID keyboard is ready, it sends the raw `HID_KEYBD_Info_TypeDef` bytes returned by the STM32 USB Host HID class.

Both payloads are followed by a CRC-16/Modbus value:

- Initial value: `0xFFFF`
- Polynomial: `0xA001` (reflected)
- Byte order on the wire: low byte, then high byte
- Check value for ASCII `123456789`: `0x4B37`

The keyboard payload is a firmware ABI structure rather than a versioned public schema. A receiver must be built against the matching STM32 USB Host HID definition. A future protocol revision should add a message type, version, and explicit payload length before compatibility is promised across firmware revisions.

## ESP32 inbound commands

TCP payloads may contain one command with optional trailing CR/LF:

| Command | Result |
| --- | --- |
| `SEND_SENSOR_DATA` | Requests an immediate measurement send |
| `RESET` | Resets the MCU only when `APP_ALLOW_REMOTE_RESET` is nonzero |

Commands must match exactly after trailing line endings are removed. The device closes the incoming link after processing it. This protocol has no authentication or encryption and is intended only for trusted networks.
