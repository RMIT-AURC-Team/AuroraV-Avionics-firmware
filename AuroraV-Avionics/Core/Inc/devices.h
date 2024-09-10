#ifndef _DEVICES_H
#define _DEVICES_H

#include "stdint.h"
#include "string.h"

#define DEVICE_NAME_LENGTH 20

typedef struct DeviceHandle_t {
  char name[DEVICE_NAME_LENGTH];
  void *device;
	struct DeviceHandle_t *ref;
} DeviceHandle_t;

DeviceHandle_t DeviceHandle_getHandle(char[DEVICE_NAME_LENGTH]);
DeviceHandle_t *DeviceHandle_getHandleRef(char[DEVICE_NAME_LENGTH]);

#endif
