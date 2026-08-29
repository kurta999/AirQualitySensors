function(add_st_target_properties target_name)
    set(app_sources
        Core/Src/bosch_sensor.c
        Core/Src/freertos.c
        Core/Src/honeywell_sensor.c
        Core/Src/light_sensor.c
        Core/Src/main.c
        Core/Src/measurement_format.c
        Core/Src/measurement_store.c
        Core/Src/protocol.c
        Core/Src/scd_sensor.c
        Core/Src/sensors.c
        Core/Src/user_interface.c
        Core/Src/usart_driver.c
        Core/Src/uv_sensor.c
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
        Core/Startup/startup_stm32l476zgtx.s
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
        STM32L476xx
        USE_FULL_LL_DRIVER
        $<$<CONFIG:Debug>:DEBUG>)

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
        -T${CMAKE_CURRENT_SOURCE_DIR}/STM32L476ZGTX_FLASH.ld)

    target_link_libraries(${target_name} PRIVATE m)

    set_target_properties(${target_name} PROPERTIES
        OUTPUT_NAME AirQualitySensors
        SUFFIX .elf
        LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/STM32L476ZGTX_FLASH.ld")

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
