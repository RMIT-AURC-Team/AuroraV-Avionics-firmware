#ifndef _BARO_H
#define _BARO_H

#include "sensors.h"
#include "stm32f439xx.h"

#define BARO_SENSITIVITY_COUNT 2
#define BARO_TEMP_SENSITIVITY  (1.0f / 65535)
#define BARO_PRESS_SENSITIVITY (1.0f / 64)
#define BARO_ODR_CFG           0x37
#define BARO_ODR_CFG_PWR       0x01
#define BARO_ODR_CFG_DEEP_DIS  0x80
#define BARO_OSR_CFG_RESERVED  0x80
#define BARO_OSR_CFG           0x36
#define BARO_OSR_CFG_PRESS_EN  0x40

#define BARO_CS GPIO_ODR_OD3

void Baro_init(Sensor_multi *, unsigned long, int, ...);
void Baro_write(Sensor_multi *, uint8_t, uint8_t);
uint8_t Baro_read(Sensor_multi *, uint8_t);

#endif
