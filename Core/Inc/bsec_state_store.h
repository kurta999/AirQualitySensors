#ifndef BSEC_STATE_STORE_H
#define BSEC_STATE_STORE_H

#include <stdint.h>

/* Returns blob length, 0 if no saved state, or -1 on an error. */
int32_t BsecState_Load(uint32_t tag, uint8_t* data, uint32_t capacity, uint32_t* sequence);
int BsecState_Save(uint32_t tag, const uint8_t* data, uint32_t length, uint32_t* sequence);

#endif
