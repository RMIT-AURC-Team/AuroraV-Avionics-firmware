#include "A3G4250D.h"

/**
 * @author Matt Ricci
 * @file A3G4250D.c
 */


/* =============================================================================== */
/**
 * @brief Initialiser for a A3G4250D gyroscope.
 * @param *gyro 			Pointer to A3G4250D struct to be initialised.
 * @param *port 			Pointer to GPIO port struct.
 * @param cs 					Device chip select address.
 * @param scale       Selected scale for read gyro rates.
 * @param *axes       Array defining sensor mounting axes.
 * @return @c NULL.
 **
 * =============================================================================== */
void A3G4250D_init(A3G4250D *gyro, GPIO_TypeDef *port, unsigned long cs, float sensitivity, const uint8_t *axes) {
  SPI_init(&gyro->base, SENSOR_GYRO, SPI1, port, cs);
  gyro->sensitivity     = sensitivity;
  gyro->axes            = axes;
  gyro->readGyro        = A3G4250D_readGyro;
  gyro->readRawBytes    = A3G4250D_readRawBytes;
  gyro->processRawBytes = A3G4250D_processRawBytes;

  A3G4250D_writeRegister(gyro, A3G4250D_CTRL_REG1, A3G4250D_CTRL_REG1_ODR_800Hz | A3G4250D_CTRL_REG1_AXIS_ENABLE | A3G4250D_CTRL_REG1_PD_ENABLE);
}

/******************************** DEVICE METHODS ********************************/

/* =============================================================================== */
/**
 * @brief Read 3-axis floating point gyro rates.
 * @param 	*gyro 		Pointer to gyro struct.
 * @param 	*out 		  Floating point gyro rate array.
 * @returns @c NULL.
 **
 * =============================================================================== */
void A3G4250D_readGyro(A3G4250D *gyro, float *out) {
  uint8_t bytes[A3G4250D_DATA_TOTAL];
  gyro->readRawBytes(gyro, bytes);
  gyro->processRawBytes(gyro, bytes, out);
}

/* =============================================================================== */
/**
 * @brief Process raw 3-axis data to floating point gyro rates.
 * @param 	*gyro 		Pointer to gyro struct.
 * @param 	*bytes 		Raw 3-axis data array.
 * @param 	*out 			Processed 3-axis data array to write.
 * @returns @c NULL.
 **
 * =============================================================================== */
void A3G4250D_processRawBytes(A3G4250D *gyro, uint8_t *bytes, float *out) {
  out[0] = gyro->sensitivity * (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]); // gyro X
  out[1] = gyro->sensitivity * (int16_t)(((uint16_t)bytes[2] << 8) | bytes[3]); // gyro Y
  out[2] = gyro->sensitivity * (int16_t)(((uint16_t)bytes[4] << 8) | bytes[5]); // gyro Z
}

/* =============================================================================== */
/**
 * @brief Read raw 3-axis data.
 * @param 	*gyro 		Pointer to gyro struct.
 * @param 	*out 			Raw 3-axis data array to write.
 * @returns @c NULL.
 **
 * =============================================================================== */
void A3G4250D_readRawBytes(A3G4250D *gyro, uint8_t *out) {
  out[0] = A3G4250D_readRegister(gyro, A3G4250D_OUT_X_H); // gyro X high
  out[1] = A3G4250D_readRegister(gyro, A3G4250D_OUT_X_L); // gyro X low
  out[2] = A3G4250D_readRegister(gyro, A3G4250D_OUT_Y_H); // gyro Y high
  out[3] = A3G4250D_readRegister(gyro, A3G4250D_OUT_Y_L); // gyro Y low
  out[4] = A3G4250D_readRegister(gyro, A3G4250D_OUT_Z_H); // gyro Z high
  out[5] = A3G4250D_readRegister(gyro, A3G4250D_OUT_Z_L); // gyro Z low
}

/******************************** INTERFACE METHODS ********************************/

void A3G4250D_writeRegister(A3G4250D *gyro, uint8_t address, uint8_t data) {
  uint16_t response;
  SPI spi = gyro->base;

  // Send write command with address and data
  uint16_t payload = (address << 0x08) | data;
  spi.port->ODR &= ~spi.cs;
  spi.send(&spi, payload);

  // Read in response from interface
  spi.receive(&spi, &response);
  spi.port->ODR |= spi.cs;
}

uint8_t A3G4250D_readRegister(A3G4250D *gyro, uint8_t address) {
  uint16_t response;
  SPI spi = gyro->base;

  // Send write command with address and data
  uint16_t payload = (address << 0x08) | 0x8000;
  spi.port->ODR &= ~spi.cs;
  spi.send(&spi, payload);

  // Read in response from interface
  spi.receive(&spi, &response);
  spi.port->ODR |= spi.cs;
  return (uint8_t)response;
}
