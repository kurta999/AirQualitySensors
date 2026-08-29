#ifndef CIRCBUFF_H
#define CIRCBUFF_H

#include <stdint.h>

typedef uint16_t circbuffsize_t;

typedef struct
{
    uint8_t* data;
    circbuffsize_t capacity;
    circbuffsize_t head;
    circbuffsize_t tail;
    circbuffsize_t count;
} circbuff_t;

void circbuff_init(circbuff_t* cb, uint8_t* storage, circbuffsize_t capacity);
uint8_t circbuff_add_byte(circbuff_t* cb, uint8_t value);
uint8_t circbuff_insert_bytes(circbuff_t* cb, const uint8_t* bytes, circbuffsize_t length);
uint8_t circbuff_get_byte(circbuff_t* cb, uint8_t* value);
uint8_t circbuff_isnext(circbuff_t* cb, const uint8_t* bytes, circbuffsize_t length,
                        uint8_t remove_if_match);
void circbuff_flush(circbuff_t* cb);
circbuffsize_t circbuff_get_data_len(circbuff_t* cb);

#endif /* CIRCBUFF_H */
