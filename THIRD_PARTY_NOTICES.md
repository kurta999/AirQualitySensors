# Third-party notices

This repository includes third-party and vendor code. Those components retain their original copyright and license terms; the repository-level MIT license does not replace them.

| Component | Location | License information |
| --- | --- | --- |
| STM32L4 HAL/LL drivers | `Drivers/STM32L4xx_HAL_Driver/` | BSD 3-Clause; see `LICENSE.txt` and `License.md` |
| CMSIS and STM32 device support | `Drivers/CMSIS/` | Apache-2.0/BSD-style component terms; see the included `LICENSE.txt` files and source headers |
| STM32 USB Host Library and generated integration | `Middlewares/ST/STM32_USB_Host_Library/`, `USB_HOST/` | ST SLA0044/Ultimate Liberty notices embedded in the source files |
| FreeRTOS kernel and CMSIS-RTOS wrapper | `Middlewares/Third_Party/FreeRTOS/` | MIT for the kernel; see `Source/LICENSE`; the CMSIS wrapper also carries ARM/ST notices in its source headers |
| Bosch Sensortec BME680 driver | `libs/bme680/` | BSD 3-Clause text embedded in each driver source/header |
| Sensirion SCD30 driver | `libs/scd30/` | BSD 3-Clause text embedded in the source/header files |
| Embedded tiny printf implementation | `libs/newlib_fixes/printf.c`, `printf.h` | MIT; copyright Marco Paland; full notice embedded in the files |

The ST7565, ESP32 integration, circular-buffer, and application-specific code in this import are treated as project code under the repository MIT license unless a source file states otherwise. When updating a bundled dependency, preserve its notices and add its upstream version or commit to this file.
