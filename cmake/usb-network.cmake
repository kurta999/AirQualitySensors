function(add_usb_network_sources target_name)
    set(tinyusb "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/TinyUSB")
    set(lwip "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/LwIP")
    file(GLOB lwip_core CONFIGURE_DEPENDS "${lwip}/src/core/*.c" "${lwip}/src/core/ipv4/*.c")
    target_sources(${target_name} PRIVATE
        ${tinyusb}/src/tusb.c
        ${tinyusb}/src/common/tusb_fifo.c
        ${tinyusb}/src/device/usbd.c
        ${tinyusb}/src/class/net/ecm_rndis_device.c
        ${tinyusb}/src/portable/synopsys/dwc2/dcd_dwc2.c
        ${tinyusb}/src/portable/synopsys/dwc2/dwc2_common.c
        ${tinyusb}/lib/networking/rndis_reports.c
        ${lwip_core}
        ${lwip}/src/netif/ethernet.c)
    target_include_directories(${target_name} PRIVATE
        ${tinyusb}/src ${tinyusb}/lib/networking ${lwip}/src/include)
endfunction()
