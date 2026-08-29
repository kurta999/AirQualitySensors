#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

uint16_t Protocol_Crc16(const uint8_t* data, size_t length);

#endif /* PROTOCOL_H */
