#ifndef _GYRO_H
#define _GYRO_H

#include "sensors.h"
#include "stm32f439xx.h"

#define GYRO_SENSITIVITY           (0.00875f)
#define GYRO_CTRL_REG1             0x20
#define GYRO_CTRL_REG1_ODR_800Hz   0xC0
#define GYRO_CTRL_REG1_PD_ENABLE   0x08
#define GYRO_CTRL_REG1_AXIS_ENABLE 0x07

#define GYRO_CS GPIO_ODR_OD2

void Gyro_init(Sensor *, unsigned long, float);
void Gyro_write(Sensor *, uint8_t, uint8_t);
uint8_t Gyro_read(Sensor *, uint8_t);

#endif
