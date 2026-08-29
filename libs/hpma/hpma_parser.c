#include "hpma/hpma_parser.h"

#include <stddef.h>

void HpmaParser_Init(HpmaParser* parser)
{
    if(parser != NULL)
    {
        parser->index = 0U;
    }
}

uint8_t HpmaParser_Push(HpmaParser* parser, uint8_t byte,
                        uint16_t* pm25_ug_m3, uint16_t* pm10_ug_m3)
{
    if(parser == NULL || pm25_ug_m3 == NULL || pm10_ug_m3 == NULL)
    {
        return 0U;
    }

    if(parser->index == 0U && byte != 0x42U)
    {
        return 0U;
    }
    if(parser->index == 1U && byte != 0x4DU)
    {
        parser->index = byte == 0x42U ? 1U : 0U;
        parser->frame[0] = byte;
        return 0U;
    }

    parser->frame[parser->index++] = byte;
    if(parser->index < HPMA_FRAME_SIZE)
    {
        return 0U;
    }
    parser->index = 0U;

    const uint16_t payload_length =
        (uint16_t)(((uint16_t)parser->frame[2] << 8U) | parser->frame[3]);
    if(payload_length != 28U)
    {
        return 0U;
    }

    uint16_t checksum = 0U;
    for(uint8_t index = 0U; index < (HPMA_FRAME_SIZE - 2U); ++index)
    {
        checksum = (uint16_t)(checksum + parser->frame[index]);
    }
    const uint16_t received_checksum =
        (uint16_t)(((uint16_t)parser->frame[30] << 8U) | parser->frame[31]);
    if(checksum != received_checksum)
    {
        return 0U;
    }

    *pm25_ug_m3 = (uint16_t)(((uint16_t)parser->frame[6] << 8U) | parser->frame[7]);
    *pm10_ug_m3 = (uint16_t)(((uint16_t)parser->frame[8] << 8U) | parser->frame[9]);
    return 1U;
}
