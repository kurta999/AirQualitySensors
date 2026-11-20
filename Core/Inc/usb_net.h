#ifndef USB_NET_H
#define USB_NET_H

#include <stdint.h>

typedef struct
{
    uint32_t initialized;
    uint32_t mounted;
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t dropped_frames;
    uint32_t dhcp_requests;
    uint32_t dhcp_replies;
    uint32_t http_requests;
    uint32_t client_connected;
    uint32_t records_sent;
    uint32_t acknowledged_bytes;
    int32_t last_tcp_error;
    const char* assertion;
} UsbNetDiagnostics;

extern volatile UsbNetDiagnostics usb_net_diagnostics;
void UsbNet_Task(void const* argument);
void UsbNet_IrqHandler(void);

#endif
