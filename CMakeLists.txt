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
set(AQS_MCU "STM32L476" CACHE STRING "Target MCU: STM32L476 or STM32L496")
set_property(CACHE AQS_MCU PROPERTY STRINGS STM32L476 STM32L496)
if(NOT AQS_MCU MATCHES "^STM32L(476|496)$")
    message(FATAL_ERROR "Unsupported AQS_MCU: ${AQS_MCU}")
endif()

if(AQS_I2C_ONLY AND AQS_WIFI_ENABLED)
    message(FATAL_ERROR "AQS_I2C_ONLY and AQS_WIFI_ENABLED are mutually exclusive")
endif()

include(cmake/st-project.cmake)

add_executable(${PROJECT_NAME})
add_st_target_properties(${PROJECT_NAME})
