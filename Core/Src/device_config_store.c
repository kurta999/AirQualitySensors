#include "app_config.h"
#include "device_config.h"
#include "bsec_state_record.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32l4xx_hal.h"
#include <string.h>

static DeviceConfig active;
static uint32_t revision;
static uint32_t sequence;
#define CONFIG_TAG UINT32_C(0x41495201)
#define CONFIG_ADDRESS UINT32_C(0x080FC000)
#define CONFIG_SLOT_SIZE 4096U
_Static_assert(sizeof(DeviceConfig) <= BSEC_STATE_PAYLOAD_CAPACITY, "Settings record capacity");

#if APP_USB_RNDIS_ENABLED
static const BsecStateRecord* slot(unsigned int i)
{ return (const BsecStateRecord*)(uintptr_t)(CONFIG_ADDRESS + i * CONFIG_SLOT_SIZE); }
static int available(void)
{
    return FLASH_SIZE == 0x100000U && (FLASH->OPTR & FLASH_OPTR_DUALBANK) != 0U &&
        (FLASH->OPTR & FLASH_OPTR_BFB2) == 0U && (SYSCFG->MEMRMP & SYSCFG_MEMRMP_FB_MODE) == 0U;
}
static int valid_slot(unsigned int i)
{
    if(!BsecStateRecord_Valid(slot(i), CONFIG_TAG) || slot(i)->length != sizeof(DeviceConfig)) { return 0; }
    DeviceConfig c;
    memcpy(&c, slot(i)->payload, sizeof(c));
    return DeviceConfig_Validate(&c);
}
static int latest_slot(void)
{
    const int a = valid_slot(0), b = valid_slot(1);
    if(!a) { return b ? 1 : -1; }
    if(!b) { return 0; }
    return (int32_t)(slot(1)->sequence - slot(0)->sequence) > 0 ? 1 : 0;
}
#endif

void DeviceConfig_Init(void)
{
    DeviceConfig_Defaults(&active);
#if APP_USB_RNDIS_ENABLED
    if(available())
    {
        const int latest = latest_slot();
        if(latest >= 0)
        {
            memcpy(&active, slot((unsigned int)latest)->payload, sizeof(active));
            sequence = slot((unsigned int)latest)->sequence;
        }
    }
#endif
    revision = 1;
}
void DeviceConfig_Get(DeviceConfig* config)
{
    taskENTER_CRITICAL();
    *config = active;
    taskEXIT_CRITICAL();
}
uint32_t DeviceConfig_Revision(void) { return revision; }
uint32_t DeviceConfig_Sequence(void) { return sequence; }

int DeviceConfig_Save(const DeviceConfig* config)
{
    if(!DeviceConfig_Validate(config)) { return -1; }
#if APP_USB_RNDIS_ENABLED
    if(!available()) { return -1; }
    if(sequence != 0U && memcmp(config, &active, sizeof(active)) == 0) { return 0; }
    const int latest = latest_slot();
    const unsigned int target = latest == 0 ? 1U : 0U;
    const uint32_t next = latest < 0 ? 1U : slot((unsigned int)latest)->sequence + 1U;
    BsecStateRecord record;
    if(BsecStateRecord_Create(&record, CONFIG_TAG, next, (const uint8_t*)config, sizeof(*config)) != 0) { return -1; }
    const uint32_t address = CONFIG_ADDRESS + target * CONFIG_SLOT_SIZE;
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_2;
    erase.Page = (address - FLASH_BASE - FLASH_BANK_SIZE) / FLASH_PAGE_SIZE;
    erase.NbPages = CONFIG_SLOT_SIZE / FLASH_PAGE_SIZE;
    uint32_t page_error;
    vTaskSuspendAll(); /* serializes settings/BSEC writes while HAL timeouts still tick */
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if(status == HAL_OK)
    {
        status = HAL_FLASHEx_Erase(&erase, &page_error);
        for(size_t offset = 0; status == HAL_OK && offset < sizeof(record); offset += 8U)
        {
            uint64_t word;
            memcpy(&word, (const uint8_t*)&record + offset, 8);
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + (uint32_t)offset, word);
        }
        (void)HAL_FLASH_Lock();
    }
    (void)xTaskResumeAll();
    if(status != HAL_OK || !valid_slot(target)) { return -1; }
    taskENTER_CRITICAL();
    active = *config;
    sequence = next;
    ++revision;
    taskEXIT_CRITICAL();
    return 0;
#else
    return -1;
#endif
}
