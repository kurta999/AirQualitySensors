#ifndef BSEC_STATE_RECORD_H
#define BSEC_STATE_RECORD_H

#include <stddef.h>
#include <stdint.h>

#define BSEC_STATE_PAYLOAD_CAPACITY 240U
#define BSEC_STATE_COMMIT UINT64_C(0x42534543434F4D54)

typedef struct
{
    uint32_t magic;
    uint32_t format;
    uint32_t sequence;
    uint32_t length;
    uint32_t configuration_tag;
    uint32_t checksum;
    uint8_t payload[BSEC_STATE_PAYLOAD_CAPACITY];
    uint64_t commit;
} BsecStateRecord;

uint32_t BsecState_Crc32(const void* data, size_t length);
int BsecStateRecord_Create(BsecStateRecord* record, uint32_t tag,
                           uint32_t sequence, const uint8_t* data, uint32_t length);
int BsecStateRecord_Valid(const BsecStateRecord* record, uint32_t tag);
int BsecStateRecord_Latest(const BsecStateRecord* first,
                           const BsecStateRecord* second, uint32_t tag);

#endif
