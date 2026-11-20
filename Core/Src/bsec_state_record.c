#include "bsec_state_record.h"

#include <string.h>

#define STATE_MAGIC UINT32_C(0x42534543)

static uint32_t crc_update(uint32_t crc, const uint8_t* data, size_t length)
{
    for(size_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for(unsigned int bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc >> 1U) ^ ((crc & 1U) ? UINT32_C(0xEDB88320) : 0U);
        }
    }
    return crc;
}

uint32_t BsecState_Crc32(const void* data, size_t length)
{
    return ~crc_update(UINT32_MAX, data, length);
}

static uint32_t record_checksum(const BsecStateRecord* record)
{
    uint32_t crc = crc_update(UINT32_MAX, (const uint8_t*)record,
                              offsetof(BsecStateRecord, checksum));
    return ~crc_update(crc, record->payload, record->length);
}

int BsecStateRecord_Create(BsecStateRecord* record, uint32_t tag,
                           uint32_t sequence, const uint8_t* data, uint32_t length)
{
    if(record == NULL || data == NULL || length == 0U || length > BSEC_STATE_PAYLOAD_CAPACITY)
    {
        return -1;
    }
    memset(record, 0xFF, sizeof(*record));
    record->magic = STATE_MAGIC;
    record->format = 1U;
    record->sequence = sequence;
    record->length = length;
    record->configuration_tag = tag;
    memcpy(record->payload, data, length);
    record->checksum = record_checksum(record);
    record->commit = BSEC_STATE_COMMIT;
    return 0;
}

int BsecStateRecord_Valid(const BsecStateRecord* record, uint32_t tag)
{
    return record != NULL && record->magic == STATE_MAGIC && record->format == 1U &&
        record->configuration_tag == tag && record->commit == BSEC_STATE_COMMIT &&
        record->length > 0U && record->length <= BSEC_STATE_PAYLOAD_CAPACITY &&
        record->checksum == record_checksum(record);
}

int BsecStateRecord_Latest(const BsecStateRecord* first,
                           const BsecStateRecord* second, uint32_t tag)
{
    const int first_valid = BsecStateRecord_Valid(first, tag);
    const int second_valid = BsecStateRecord_Valid(second, tag);
    if(!first_valid) { return second_valid ? 1 : -1; }
    if(!second_valid) { return 0; }
    return (int32_t)(second->sequence - first->sequence) > 0 ? 1 : 0;
}
