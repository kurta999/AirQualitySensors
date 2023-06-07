#include "stm32l4xx_hal.h"

#ifdef DEBUG
#define DEBUG_VOLATILE volatile
#else
#define	DEBUG_VOLATILE
#endif

/* Helper macro for acquiring array length */
#define ARRAY_LEN(x) \
  (sizeof(x) / sizeof((x)[0]))
