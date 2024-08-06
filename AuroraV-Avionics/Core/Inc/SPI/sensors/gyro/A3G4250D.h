/**
 * @author Matt Ricci
 * @ingroup Sensors
 * @addtogroup Gyroscope
 * @file A3G4250D.h
 */

#ifndef _A3G4250D_H
#define _A3G4250D_H

#include "SPI/spi.h"
#include "stm32f439xx.h"

#define A3G4250D_SENSITIVITY           (0.00875f)
#define A3G4250D_CTRL_REG1             0x20
#define A3G4250D_CTRL_REG1_ODR_800Hz   0xC0
#define A3G4250D_CTRL_REG1_PD_ENABLE   0x08
#define A3G4250D_CTRL_REG1_AXIS_ENABLE 0x07
#define A3G4250D_OUT_X_L               0x28
#define A3G4250D_OUT_X_H               0x29
#define A3G4250D_OUT_Y_L               0x2A
#define A3G4250D_OUT_Y_H               0x2B
#define A3G4250D_OUT_Z_L               0x2C
#define A3G4250D_OUT_Z_H               0x2D

#define A3G4250D_DATA_SIZE  2 // Two bytes per axis
#define A3G4250D_DATA_COUNT 3 // Three axes - X Y Z
#define A3G4250D_DATA_TOTAL (A3G4250D_DATA_COUNT * A3G4250D_DATA_SIZE)

/**
 * @ingroup Gyroscope
 * @addtogroup A3G4250D
 * @{
 */

/** @extends SPI */
typedef struct A3G4250D {
  SPI base;                                                       //!< Parent SPI interface
  float sensitivity;                                              //!< Gyroscope sensitivity
  const uint8_t *axes;                                            //!< Array defining axes of mounting
  void (*readGyro)(struct A3G4250D *, float *);                   //!< Gyro read method.         @see A3G4250D_readGyro
  void (*readRawBytes)(struct A3G4250D *, uint8_t *);             //!< Raw gyro read method.     @see A3G4250D_readRawBytes
  void (*processRawBytes)(struct A3G4250D *, uint8_t *, float *); //!< Process raw gyro method.  @see A3G4250D_processRawBytes
} A3G4250D;

void A3G4250D_init(A3G4250D *, GPIO_TypeDef *, unsigned long, const float, const uint8_t *);
void A3G4250D_readGyro(A3G4250D *, float *);
void A3G4250D_readRawBytes(A3G4250D *, uint8_t *);
void A3G4250D_processRawBytes(A3G4250D *, uint8_t *, float *);

uint8_t A3G4250D_readRegister(A3G4250D *, uint8_t);
void A3G4250D_writeRegister(A3G4250D *, uint8_t, uint8_t);

/** @} */
#endif