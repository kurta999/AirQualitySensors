cmake_minimum_required(VERSION 3.20)

project(AirQualitySensors VERSION 0.1.0 LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(AQS_WIFI_ENABLED
    "Compile the ESP32 Wi-Fi code path with placeholder credentials" OFF)
option(AQS_I2C_ONLY "Enable only TCS34725, VEML6070 and SCD30 on the Nucleo I2C1 bus" OFF)
option(AQS_HARDWARE_ENABLED "Default enable state for application hardware" ON)
option(AQS_BME680_ENABLED "Enable the BME680 alongside the minimal Nucleo profile" OFF)
option(AQS_BSEC_ENABLED "Calculate Bosch IAQ using the locally installed BSEC library" OFF)
option(AQS_USB_RNDIS_ENABLED "Use USB RNDIS Ethernet and DHCP instead of Wi-Fi" OFF)
set(BSEC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/libs/bsec" CACHE PATH "Bosch BSEC package directory")
set(AQS_MCU "STM32L476" CACHE STRING "Target MCU: STM32L476 or STM32L496")
set_property(CACHE AQS_MCU PROPERTY STRINGS STM32L476 STM32L496)
if(NOT AQS_MCU MATCHES "^STM32L(476|496)$")
    message(FATAL_ERROR "Unsupported AQS_MCU: ${AQS_MCU}")
endif()

if(AQS_I2C_ONLY AND AQS_WIFI_ENABLED)
    message(FATAL_ERROR "AQS_I2C_ONLY and AQS_WIFI_ENABLED are mutually exclusive")
endif()
if(AQS_BSEC_ENABLED AND (NOT AQS_BME680_ENABLED OR NOT AQS_MCU STREQUAL "STM32L496"))
    message(FATAL_ERROR "BSEC currently requires AQS_BME680_ENABLED=ON and AQS_MCU=STM32L496")
endif()

include(cmake/st-project.cmake)
include(cmake/usb-network.cmake)

add_executable(${PROJECT_NAME})
add_st_target_properties(${PROJECT_NAME})
