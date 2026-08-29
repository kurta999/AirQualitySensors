#include "circbuff.h"

#include <stddef.h>

#ifndef CIRCBUFF_HOST_TEST
#include "device_specific.h"

static uint32_t circbuff_lock(void)
{
    const uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    return interrupt_state;
}

static void circbuff_unlock(uint32_t interrupt_state)
{
    if(interrupt_state == 0U)
    {
        __enable_irq();
    }
}
#else
static uint32_t circbuff_lock(void)
{
    return 0U;
}

static void circbuff_unlock(uint32_t interrupt_state)
{
    (void)interrupt_state;
}
#endif

void circbuff_init(circbuff_t* cb, uint8_t* storage, circbuffsize_t capacity)
{
    if(cb == NULL)
    {
        return;
    }

    const uint32_t interrupt_state = circbuff_lock();
    cb->data = storage;
    cb->capacity = capacity;
    cb->head = 0U;
    cb->tail = 0U;
    cb->count = 0U;
    circbuff_unlock(interrupt_state);
}

uint8_t circbuff_add_byte(circbuff_t* cb, uint8_t value)
{
    return circbuff_insert_bytes(cb, &value, 1U);
}

uint8_t circbuff_insert_bytes(circbuff_t* cb, const uint8_t* bytes, circbuffsize_t length)
{
    if(cb == NULL || bytes == NULL || cb->data == NULL || cb->capacity == 0U)
    {
        return 0U;
    }

    const uint32_t interrupt_state = circbuff_lock();
    const circbuffsize_t available = (circbuffsize_t)(cb->capacity - cb->count);
    if(length > available)
    {
        circbuff_unlock(interrupt_state);
        return 0U;
    }

    for(circbuffsize_t index = 0U; index < length; ++index)
    {
        cb->data[cb->head] = bytes[index];
        cb->head = (circbuffsize_t)((cb->head + 1U) % cb->capacity);
    }
    cb->count = (circbuffsize_t)(cb->count + length);
    circbuff_unlock(interrupt_state);
    return 1U;
}

uint8_t circbuff_get_byte(circbuff_t* cb, uint8_t* value)
{
    if(cb == NULL || value == NULL || cb->data == NULL || cb->capacity == 0U)
    {
        return 0U;
    }

    const uint32_t interrupt_state = circbuff_lock();
    if(cb->count == 0U)
    {
        circbuff_unlock(interrupt_state);
        return 0U;
    }

    *value = cb->data[cb->tail];
    cb->tail = (circbuffsize_t)((cb->tail + 1U) % cb->capacity);
    --cb->count;
    circbuff_unlock(interrupt_state);
    return 1U;
}

uint8_t circbuff_isnext(circbuff_t* cb, const uint8_t* bytes, circbuffsize_t length,
                        uint8_t remove_if_match)
{
    if(cb == NULL || bytes == NULL || cb->data == NULL || cb->capacity == 0U)
    {
        return 0U;
    }

    const uint32_t interrupt_state = circbuff_lock();
    if(length > cb->count)
    {
        circbuff_unlock(interrupt_state);
        return 0U;
    }

    circbuffsize_t position = cb->tail;
    for(circbuffsize_t index = 0U; index < length; ++index)
    {
        if(cb->data[position] != bytes[index])
        {
            circbuff_unlock(interrupt_state);
            return 0U;
        }
        position = (circbuffsize_t)((position + 1U) % cb->capacity);
    }

    if(remove_if_match != 0U)
    {
        cb->tail = position;
        cb->count = (circbuffsize_t)(cb->count - length);
    }
    circbuff_unlock(interrupt_state);
    return 1U;
}

void circbuff_flush(circbuff_t* cb)
{
    if(cb == NULL)
    {
        return;
    }

    const uint32_t interrupt_state = circbuff_lock();
    cb->head = 0U;
    cb->tail = 0U;
    cb->count = 0U;
    circbuff_unlock(interrupt_state);
}

circbuffsize_t circbuff_get_data_len(circbuff_t* cb)
{
    if(cb == NULL)
    {
        return 0U;
    }

    const uint32_t interrupt_state = circbuff_lock();
    const circbuffsize_t count = cb->count;
    circbuff_unlock(interrupt_state);
    return count;
}
