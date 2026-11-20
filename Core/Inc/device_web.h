#ifndef DEVICE_WEB_H
#define DEVICE_WEB_H
struct netif;
void DeviceWeb_Start(struct netif* interface);
void DeviceWeb_Service(void);
#endif
