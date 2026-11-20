function(add_st_target_properties target_name)
    string(TOLOWER "${AQS_MCU}" mcu_lower)
    set(linker_script "${CMAKE_CURRENT_SOURCE_DIR}/${AQS_MCU}ZGTX_FLASH.ld")
    set(app_sources
        Core/Src/app_hardware.c
        Core/Src/device_config.c
        Core/Src/device_config_store.c
        Core/Src/http_request.c
        Core/Src/measurement_history.c
        Core/Src/bosch_sensor.c
        Core/Src/bsec_processing.c
        Core/Src/bsec_state_record.c
        Core/Src/bsec_state_store.c
        Core/Src/freertos.c
        Core/Src/honeywell_sensor.c
        Core/Src/i2c_bus.c
        Core/Src/light_sensor.c
        Core/Src/main.c
        Core/Src/measurement_format.c
        Core/Src/measurement_store.c
        Core/Src/measurement_view.c
        Core/Src/protocol.c
        Core/Src/scd_sensor.c
        Core/Src/sensors.c
        Core/Src/task_health.c
        Core/Src/user_interface.c
        Core/Src/usart_driver.c
        Core/Src/uv_sensor.c
        Core/Src/usb_dhcp.c
        Core/Src/usb_net.c
        Core/Src/usb_net_descriptors.c
        Core/Src/wifi.c
        libs/ESP32/esp32.c
        libs/circbuff/circbuff.c
        libs/hpma/hpma_parser.c
        libs/newlib_fixes/printf.c
        libs/newlib_fixes/time_fixes.c
        libs/st7565/ST7565.c)

    set(generated_sources
        USB_HOST/App/usb_host.c
        USB_HOST/Target/usbh_conf.c
        USB_HOST/Target/usbh_platform.c
        libs/scd30/sensirion_common.c
        libs/scd30/scd30.c
        libs/bme680/bme680.c
        Core/Src/adc.c
        Core/Src/dma.c
        Core/Src/gpio.c
        Core/Src/i2c.c
        Core/Src/iwdg.c
        Core/Src/spi.c
        Core/Src/stm32l4xx_hal_msp.c
        Core/Src/stm32l4xx_hal_timebase_tim.c
        Core/Src/stm32l4xx_it.c
        Core/Src/syscalls.c
        Core/Src/sysmem.c
        Core/Src/system_stm32l4xx.c
        Core/Src/tim.c
        Core/Src/usart.c
        Core/Startup/startup_${mcu_lower}zgtx.s
        Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.c
        Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.c
        Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.c
        Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.c
        Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.c
        Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c
        Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.c
        Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.c
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
        Middlewares/Third_Party/FreeRTOS/Source/croutine.c
        Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
        Middlewares/Third_Party/FreeRTOS/Source/list.c
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
        Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_1.c
        Middlewares/Third_Party/FreeRTOS/Source/queue.c
        Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
        Middlewares/Third_Party/FreeRTOS/Source/tasks.c
        Middlewares/Third_Party/FreeRTOS/Source/timers.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_hcd.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_iwdg.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rcc.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usart.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c)

    target_sources(${target_name} PRIVATE ${app_sources} ${generated_sources})

    target_compile_definitions(${target_name} PRIVATE
        USE_HAL_DRIVER
        ${AQS_MCU}xx
        USE_FULL_LL_DRIVER
        $<$<CONFIG:Debug>:DEBUG>)

    if(AQS_I2C_ONLY)
        target_compile_definitions(${target_name} PRIVATE
            APP_CONFIG_IGNORE_LOCAL APP_HARDWARE_ENABLED=0
            APP_VEML6070_ENABLED=1 APP_TCS34725_ENABLED=1 APP_SCD30_ENABLED=1
            APP_WIFI_ENABLED=0
            APP_SCD30_RECOVERY_ENABLED=1
            APP_I2C1_INTERNAL_PULLUPS=1 APP_I2C1_TIMING=0x3090F9F9U)
    elseif(NOT AQS_HARDWARE_ENABLED)
        target_compile_definitions(${target_name} PRIVATE
            APP_CONFIG_IGNORE_LOCAL APP_HARDWARE_ENABLED=0 APP_WIFI_ENABLED=0)
    endif()

    if(AQS_BME680_ENABLED)
        target_compile_definitions(${target_name} PRIVATE APP_BME680_ENABLED=1)
    endif()

    if(AQS_BSEC_ENABLED)
        set(bsec_archive "${BSEC_ROOT}/src/cortex-m4/fpv4-sp-d16-hard/libalgobsec.a")
        if(NOT EXISTS "${bsec_archive}" OR NOT EXISTS "${BSEC_ROOT}/src/inc/bsec_interface.h")
            message(FATAL_ERROR "Missing Bosch BSEC package. Run scripts/Install-Bsec.ps1 first.")
        endif()
        file(SHA256 "${bsec_archive}" bsec_hash)
        if(NOT bsec_hash STREQUAL "ad831fcb676e7cafd491542b2277b773113ce5f92ec022945bebb53dd8595650")
            message(FATAL_ERROR "Unexpected BSEC binary; install the pinned package with scripts/Install-Bsec.ps1")
        endif()
        add_library(BoschBsec STATIC IMPORTED)
        set_target_properties(BoschBsec PROPERTIES IMPORTED_LOCATION "${bsec_archive}")
        target_link_libraries(${target_name} PRIVATE BoschBsec)
        target_compile_definitions(${target_name} PRIVATE APP_BSEC_ENABLED=1)
        target_include_directories(${target_name} PRIVATE "${BSEC_ROOT}/src/inc"
            "${BSEC_ROOT}/src/config/bme680/bme680_iaq_33v_3s_4d")
    endif()

    if(AQS_USB_RNDIS_ENABLED)
        if(NOT AQS_I2C_ONLY OR AQS_WIFI_ENABLED OR NOT AQS_MCU STREQUAL "STM32L496")
            message(FATAL_ERROR "USB RNDIS requires the minimal L496 profile with Wi-Fi off")
        endif()
        target_compile_definitions(${target_name} PRIVATE APP_USB_RNDIS_ENABLED=1)
        add_usb_network_sources(${target_name})
    endif()

    # Compile-only Wi-Fi configuration. The default build leaves APP_WIFI_ENABLED
    # at 0, so without this the whole ESP32 code path is never compiled by CI.
    # APP_CONFIG_IGNORE_LOCAL makes the configuration reproducible by ignoring
    # any app_config.local.h in the working tree; the credentials are
    # placeholders and this build is not meant to connect to anything.
    if(AQS_WIFI_ENABLED)
        target_compile_definitions(${target_name} PRIVATE
            APP_CONFIG_IGNORE_LOCAL
            APP_WIFI_ENABLED=1
            APP_WIFI_SSID="ci-build-only"
            APP_WIFI_PASSWORD="ci-build-only")
    endif()

    target_include_directories(${target_name} PRIVATE
        Core/Inc
        Drivers/STM32L4xx_HAL_Driver/Inc
        Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32L4xx/Include
        Drivers/CMSIS/Include
        USB_HOST/App
        USB_HOST/Target
        Middlewares/ST/STM32_USB_Host_Library/Core/Inc
        Middlewares/ST/STM32_USB_Host_Library/Class/HID/Inc
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        libs)

    target_compile_options(${target_name} PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -fdata-sections
        -ffunction-sections
        --specs=nano.specs
        $<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>
        $<$<CONFIG:Debug>:-O0>
        $<$<CONFIG:Debug>:-g3>
        $<$<CONFIG:Release>:-Os>
        $<$<CONFIG:Release>:-g0>)

    set_source_files_properties(${app_sources} PROPERTIES
        COMPILE_OPTIONS "-Wall;-Wextra;-Wformat=2;-Wshadow;-Wundef;-Wdouble-promotion")

    target_link_options(${target_name} PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        --specs=nano.specs
        --specs=nosys.specs
        -Wl,--gc-sections
        -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${target_name}.map
        -T${linker_script})

    target_link_libraries(${target_name} PRIVATE m)

    set_target_properties(${target_name} PROPERTIES
        OUTPUT_NAME AirQualitySensors
        SUFFIX .elf
        LINK_DEPENDS "${linker_script}")

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target_name}>
        COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${target_name}> ${CMAKE_CURRENT_BINARY_DIR}/AirQualitySensors.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${target_name}> ${CMAKE_CURRENT_BINARY_DIR}/AirQualitySensors.bin
        BYPRODUCTS
            ${CMAKE_CURRENT_BINARY_DIR}/AirQualitySensors.hex
            ${CMAKE_CURRENT_BINARY_DIR}/AirQualitySensors.bin
        VERBATIM)
endfunction()
