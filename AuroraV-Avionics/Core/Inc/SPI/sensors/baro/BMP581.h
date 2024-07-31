/**
 * @author Matt Ricci
 * @ingroup Sensors
 * @addtogroup Barometer
 * @file BMP581.h
 */

#ifndef _BMP581_H
#define _BMP581_H

#include "SPI/spi.h"
#include "stm32f439xx.h"

#define BMP581_TEMP_SENSITIVITY  (1.0f / 65535)
#define BMP581_PRESS_SENSITIVITY (1.0f / 64)
#define BMP581_ODR_CFG           0x37
#define BMP581_ODR_CFG_PWR       0x01
#define BMP581_ODR_CFG_DEEP_DIS  0x80
#define BMP581_OSR_CFG_RESERVED  0x80
#define BMP581_OSR_CFG           0x36
#define BMP581_OSR_CFG_PRESS_EN  0x40
#define BMP581_TEMPERATURE_XLSB  0x1D
#define BMP581_TEMPERATURE_LSB   0x1E
#define BMP581_TEMPERATURE_MSB   0x1F
#define BMP581_PRESSURE_XLSB     0x20
#define BMP581_PRESSURE_LSB      0x21
#define BMP581_PRESSURE_MSB      0x22

#define BMP581_DATA_SIZE  3 // Three bytes per reading
#define BMP581_DATA_COUNT 2 // Two readings - temperature, pressure
#define BMP581_DATA_TOTAL (BMP581_DATA_COUNT * BMP581_DATA_SIZE)

/**
 * @ingroup Barometer
 * @defgroup BMP581
 * @addtogroup BMP581
 * @todo Fill in implementation documentation in BMP581.c
 * @todo Document interface in BMP581.h
 * @{
 */

/** @extends SPI */
typedef struct BMP581 {
  SPI base;
  float pressSensitivity;
  float tempSensitivity;
  void (*readTemp)(struct BMP581 *, float *);
  void (*readPress)(struct BMP581 *, float *);
  void (*readRawTemp)(struct BMP581 *, uint8_t *);
  void (*readRawPress)(struct BMP581 *, uint8_t *);
  void (*processRawTemp)(struct BMP581 *, uint8_t *, float *);
  void (*processRawPress)(struct BMP581 *, uint8_t *, float *);
} BMP581;

void BMP581_init(BMP581 *, GPIO_TypeDef *, unsigned long, const float, const float);
void BMP581_readTemp(BMP581 *, float *);
void BMP581_readPress(BMP581 *, float *);
void BMP581_readRawTemp(BMP581 *, uint8_t *);
void BMP581_readRawPress(BMP581 *, uint8_t *);
void BMP581_processRawTemp(BMP581 *, uint8_t *, float *);
void BMP581_processRawPress(BMP581 *, uint8_t *, float *);

uint8_t BMP581_readRegister(BMP581 *, uint8_t);
void BMP581_writeRegister(BMP581 *, uint8_t, uint8_t);

/** @} */
#endif
