#include "protocol.h"

uint16_t Protocol_Crc16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    if(data == NULL)
    {
        return crc;
    }

    for(size_t index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for(uint8_t bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 1U) != 0U
                ? (uint16_t)((crc >> 1U) ^ 0xA001U)
                : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}
