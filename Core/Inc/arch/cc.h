#ifndef AQS_LWIP_ARCH_CC_H
#define AQS_LWIP_ARCH_CC_H

#include <stdint.h>
typedef int sys_prot_t;
void UsbNet_Assert(const char* message);
uint32_t UsbNet_Random(void);
#define LWIP_PLATFORM_ASSERT(message) UsbNet_Assert(message)
#define LWIP_PLATFORM_DIAG(message) do { } while(0)
#define LWIP_RAND() UsbNet_Random()
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x

#endif
