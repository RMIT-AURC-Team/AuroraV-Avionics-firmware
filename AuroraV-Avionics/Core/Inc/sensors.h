#ifndef _SENSORS_H
#define _SENSORS_H

#include "drivers.h"

#define GYRO_SENSITIVITY           (0.00875f)
#define GYRO_CTRL_REG1             0x20
#define GYRO_CTRL_REG1_ODR_800Hz   0xC0
#define GYRO_CTRL_REG1_PD_ENABLE   0x08
#define GYRO_CTRL_REG1_AXIS_ENABLE 0x07

#define MAGNET_SENSITIVITY    (1.0f / 1711.0f)
#define MAGNET_CTRL_REG1      0x20
#define MAGNET_CTRL_REG1_FAST 0x02
#define MAGNET_CTRL_REG2      0x21
#define MAGNET_CTRL_REG2_FS16 0x60

#define ACCEL_SENSITIVITY_32G (1.0f / 1024.0f)
#define ACCEL_SENSITIVITY_16G (1.0f / 2048.0f)
#define ACCEL_CNTL1           0x1B
#define ACCEL_CNTL1_PC1       0x80
#define ACCEL_CNTL1_RES       0x40
#define ACCEL_CNTL1_GSEL_32G  0x10
#define ACCEL_CNTL1_GSEL_16G  0x08
#define ACCEL_CNTL1_GSEL_8G   0x00
#define ACCEL_ODCNTL          0x21
#define ACCEL_ODCNTL_RESERVED 0x90

#define BARO_TEMP_SENSITIVITY  (1.0f / 65535)
#define BARO_PRESS_SENSITIVITY (1.0f / 64)
#define BARO_ODR_CFG           0x37
#define BARO_ODR_CFG_PWR       0x01
#define BARO_ODR_CFG_DEEP_DIS  0x80
#define BARO_OSR_CFG_RESERVED  0x80
#define BARO_OSR_CFG           0x36
#define BARO_OSR_CFG_PRESS_EN  0x40

void configure_Sensors();
void write_GYRO(uint8_t, uint8_t);
uint8_t read_GYRO(uint8_t);

uint8_t read_BARO(uint8_t);
void write_BARO(uint8_t, uint8_t);

void write_MAG(uint8_t, uint8_t);
uint8_t read_MAG(uint8_t);

void write_ACCEL_1(uint8_t, uint8_t);
uint8_t read_ACCEL_1(uint8_t);
void write_ACCEL_2(uint8_t, uint8_t);
uint8_t read_ACCEL_2(uint8_t);

#endif
