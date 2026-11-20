#include "usb_dhcp.h"

#include <string.h>

static const uint8_t cookie[4] = {99U, 130U, 83U, 99U};
static const uint8_t zero_ip[4] = {0U};

size_t UsbDhcp_Reply(const uint8_t* request, size_t length,
    uint8_t* reply, size_t capacity, const uint8_t server[4],
    const uint8_t client[4], const uint8_t mask[4])
{
    if(request == NULL || reply == NULL || server == NULL || client == NULL || mask == NULL ||
       length < 240U || capacity < 300U || request[0] != 1U || request[1] != 1U ||
       request[2] != 6U || memcmp(request + 236U, cookie, 4U) != 0 ||
       memcmp(request + 24U, zero_ip, 4U) != 0)
    {
        return 0U;
    }
    uint8_t type = 0U;
    const uint8_t* requested_ip = NULL;
    for(size_t pos = 240U; pos < length; )
    {
        const uint8_t option = request[pos++];
        if(option == 255U) { break; }
        if(option == 0U) { continue; }
        if(pos >= length) { return 0U; }
        const size_t size = request[pos++];
        if(size > length - pos) { return 0U; }
        if(option == 53U)
        {
            if(size != 1U) { return 0U; }
            type = request[pos];
        }
        if(option == 50U)
        {
            if(size != 4U) { return 0U; }
            requested_ip = request + pos;
        }
        if(option == 54U && (size != 4U || memcmp(request + pos, server, 4U) != 0))
        {
            return 0U;
        }
        pos += size;
    }
    if(type != 1U && type != 3U && type != 8U) { return 0U; }
    uint8_t response_type = type == 1U ? 2U : 5U;
    if(type == 3U && ((requested_ip != NULL && memcmp(requested_ip, client, 4U) != 0) ||
       (memcmp(request + 12U, zero_ip, 4U) != 0 && memcmp(request + 12U, client, 4U) != 0)))
    {
        response_type = 6U;
    }
    memset(reply, 0, 300U);
    reply[0] = 2U;
    reply[1] = 1U;
    reply[2] = 6U;
    memcpy(reply + 4U, request + 4U, 4U);
    reply[10] = 0x80U; /* Broadcast reply while the host has no address yet. */
    if(type != 8U && response_type != 6U) { memcpy(reply + 16U, client, 4U); }
    memcpy(reply + 20U, server, 4U);
    memcpy(reply + 28U, request + 28U, 16U);
    memcpy(reply + 236U, cookie, 4U);
    size_t pos = 240U;
    reply[pos++] = 53U; reply[pos++] = 1U; reply[pos++] = response_type;
    reply[pos++] = 54U; reply[pos++] = 4U;
    memcpy(reply + pos, server, 4U); pos += 4U;
    if(response_type != 6U)
    {
        reply[pos++] = 1U; reply[pos++] = 4U;
        memcpy(reply + pos, mask, 4U); pos += 4U;
        /* One day lease, renewal at 12 h, rebinding at 21 h. */
        static const uint8_t timers[] = {51,4,0,1,81,128,58,4,0,0,168,192,59,4,0,1,39,80};
        memcpy(reply + pos, timers, sizeof(timers)); pos += sizeof(timers);
        reply[pos++] = 26U; reply[pos++] = 2U; reply[pos++] = 5U; reply[pos++] = 220U;
    }
    reply[pos] = 255U;
    return 300U;
}
