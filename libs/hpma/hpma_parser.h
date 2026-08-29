#ifndef HPMA_PARSER_H
#define HPMA_PARSER_H

#include <stdint.h>

#define HPMA_FRAME_SIZE 32U

typedef struct
{
    uint8_t frame[HPMA_FRAME_SIZE];
    uint8_t index;
} HpmaParser;

void HpmaParser_Init(HpmaParser* parser);
uint8_t HpmaParser_Push(HpmaParser* parser, uint8_t byte,
                        uint16_t* pm25_ug_m3, uint16_t* pm10_ug_m3);

#endif /* HPMA_PARSER_H */
