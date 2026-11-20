#ifndef USB_DHCP_H
#define USB_DHCP_H

#include <stddef.h>
#include <stdint.h>

/* A single-host USB link. No gateway or DNS settings are advertised. */
size_t UsbDhcp_Reply(const uint8_t* request, size_t request_length,
    uint8_t* reply, size_t reply_capacity, const uint8_t server[4],
    const uint8_t client[4], const uint8_t mask[4]);

#endif
