

#ifndef STM32_ATOMIC_H_
#define STM32_ATOMIC_H_

#include "device_specific.h"

static __inline__ uint8_t __iDisableIrqRetval(void)
{
    __disable_irq();
    return 1;
}

static __inline__ void __iRestore(const uint32_t *__primask)
{
  if(!*__primask)
    __enable_irq();
  __asm__ volatile ("" ::: "memory");
}

/**
 * @brief This is for now the only possible parameter for ATOMIC_BLOCK.
 *        It will cause the ATOMIC_BLOCK to restore the previous state of
 *        the IRQ.
 */
#define ATOMIC_RESTORESTATE uint32_t primask_save \
  __attribute__((__cleanup__(__iRestore))) = __get_PRIMASK()

/**
 * @brief Creates a block of code that is guaranteed to be executed
          atomically. Upon entering the block the IRQ is disabled,
          and re-enabled upon exiting the block from any exit path.
 */
#define ATOMIC_BLOCK(type) for ( type, __ToDo = __iDisableIrqRetval(); \
                         __ToDo ; __ToDo = 0 )

#endif /* STM32_ATOMIC_H_ */
