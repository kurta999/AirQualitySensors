#include "i2c_bus.h"

#include <stddef.h>

#include "cmsis_os.h"

static osMutexId i2c_bus_mutex;

void I2cBus_Init(void)
{
    if(i2c_bus_mutex != NULL)
    {
        return;
    }

    osMutexDef(i2c_bus);
    i2c_bus_mutex = osMutexCreate(osMutex(i2c_bus));
}

HAL_StatusTypeDef I2cBus_Lock(void)
{
    if(i2c_bus_mutex == NULL)
    {
        return HAL_ERROR;
    }
    return osMutexWait(i2c_bus_mutex, I2C_BUS_LOCK_TIMEOUT_MS) == osOK
        ? HAL_OK
        : HAL_BUSY;
}

void I2cBus_Unlock(void)
{
    if(i2c_bus_mutex != NULL)
    {
        (void)osMutexRelease(i2c_bus_mutex);
    }
}
