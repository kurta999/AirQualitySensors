#include "app_config.h"

#if APP_BSEC_ENABLED
#include "bsec_state_store.h"
#include "bsec_state_record.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32l4xx_hal.h"

#include <string.h>

/* Last 8 KiB are excluded from the L496 linker region. Each slot owns 4 KiB. */
#define STATE_FIRST_ADDRESS UINT32_C(0x080FE000)
#define STATE_SLOT_SPAN 4096U

_Static_assert(sizeof(BsecStateRecord) % 8U == 0U, "Flash doubleword alignment");
_Static_assert(offsetof(BsecStateRecord, commit) + 8U == sizeof(BsecStateRecord), "Commit must be last");

static const BsecStateRecord* slot(unsigned int index)
{
    return (const BsecStateRecord*)(uintptr_t)(STATE_FIRST_ADDRESS + index * STATE_SLOT_SPAN);
}

static int storage_available(void)
{
    return FLASH_SIZE == 0x100000U &&
        (FLASH->OPTR & FLASH_OPTR_DUALBANK) != 0U &&
        (FLASH->OPTR & FLASH_OPTR_BFB2) == 0U &&
        (SYSCFG->MEMRMP & SYSCFG_MEMRMP_FB_MODE) == 0U;
}

int32_t BsecState_Load(uint32_t tag, uint8_t* data, uint32_t capacity, uint32_t* sequence)
{
    if(data == NULL || sequence == NULL || !storage_available()) { return -1; }
    const int index = BsecStateRecord_Latest(slot(0U), slot(1U), tag);
    if(index < 0) { return 0; }
    const BsecStateRecord* record = slot((unsigned int)index);
    if(record->length > capacity) { return -1; }
    memcpy(data, record->payload, record->length);
    *sequence = record->sequence;
    return (int32_t)record->length;
}

int BsecState_Save(uint32_t tag, const uint8_t* data, uint32_t length, uint32_t* sequence)
{
    /* Only erase the verified 1-MiB, dual-bank, unswapped L496 layout. */
    if(sequence == NULL || !storage_available())
    {
        return -1;
    }
    const int latest = BsecStateRecord_Latest(slot(0U), slot(1U), tag);
    const unsigned int target = latest == 0 ? 1U : 0U;
    const uint32_t next_sequence = latest < 0 ? 1U : slot((unsigned int)latest)->sequence + 1U;
    BsecStateRecord record;
    if(BsecStateRecord_Create(&record, tag, next_sequence, data, length) != 0) { return -1; }

    const uint32_t address = STATE_FIRST_ADDRESS + target * STATE_SLOT_SPAN;
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_2;
    erase.Page = (address - FLASH_BASE - FLASH_BANK_SIZE) / FLASH_PAGE_SIZE;
    erase.NbPages = STATE_SLOT_SPAN / FLASH_PAGE_SIZE;
    uint32_t page_error = 0U;
    HAL_StatusTypeDef status;
    /* Bank 1 code keeps running while bank 2 is programmed. Keep the HAL
     * tick interrupt alive so its flash-operation timeouts remain bounded. */
    vTaskSuspendAll();
    status = HAL_FLASH_Unlock();
    if(status == HAL_OK)
    {
        status = HAL_FLASHEx_Erase(&erase, &page_error);
        for(size_t offset = 0U; status == HAL_OK && offset < sizeof(record); offset += 8U)
        {
            uint64_t word;
            memcpy(&word, (const uint8_t*)&record + offset, sizeof(word));
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + (uint32_t)offset, word);
        }
        (void)HAL_FLASH_Lock();
    }
    (void)xTaskResumeAll();
    if(status != HAL_OK || !BsecStateRecord_Valid(slot(target), tag)) { return -1; }
    *sequence = next_sequence;
    return 0;
}
#endif
