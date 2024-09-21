/***********************************************************************************
 * @file        BMP581.c                                                           *
 * @author      Matt Ricci                                                         *
 * @addtogroup  BMP581                                                             *
 *                                                                                 *
 * @todo Add altitude calculation method                                           *
 * @todo Document implementation                                                   *
 * @todo Move private interface methods (read/write register) to static functions  *
 *       with internal prototypes.                                                 *
 * @todo Replace giga loop with hardware timer                                     *
 * @{                                                                              *
 ***********************************************************************************/

#include "BMP581.h"

/* =============================================================================== */
/**
 * @brief Initialiser for a BMP581 barometer.
 * @param *baro 						Pointer to BMP581 struct to be initialised.
 * @param *port 						Pointer to GPIO port struct.
 * @param cs 								Device chip select address.
 * @param tempSensitivity   Barometer temperature sensitivity.
 * @param pressSensitivity  Barometer pressure sensitivity.
 * @return @c NULL.
 **
 * =============================================================================== */
DeviceHandle_t BMP581_init(
    BMP581 *baro,
    char name[DEVICE_NAME_LENGTH],
    GPIO_TypeDef *port,
    unsigned long cs,
    float tempSensitivity,
    float pressSensitivity
) {
  SPI_init(&baro->base, SENSOR_BARO, SPI1, MODE8, port, cs);
  baro->tempSensitivity     = tempSensitivity;
  baro->pressSensitivity    = pressSensitivity;
  baro->update              = BMP581_update;
  baro->readTemp            = BMP581_readTemp;
  baro->readRawTemp         = BMP581_readRawTemp;
  baro->processRawTemp      = BMP581_processRawTemp;
  baro->readPress           = BMP581_readPress;
  baro->readRawPress        = BMP581_readRawPress;
  baro->processRawPress     = BMP581_processRawPress;

	uint8_t chipID = 0;
  chipID = BMP581_readRegister(baro, 0x01);
	
	volatile uint8_t counter  = 0;
	
  BMP581_writeRegister(baro, BMP581_ODR_CFG, BMP581_ODR_CFG_DEEP_DIS); 				// Disable deep sleep  
  for (uint32_t i = 0; i < 0xFFFFFF; i++) {counter++;}												// Wait for at least t_standby
  BMP581_writeRegister(baro, BMP581_ODR_CFG, BMP581_ODR_CFG_PWR_CONTINUOUS);  // Set continuous sample


  uint8_t OSRCFG = BMP581_readRegister(baro, BMP581_OSR_CFG);
  BMP581_writeRegister(baro, BMP581_OSR_CFG, (BMP581_OSR_CFG_RESERVED & OSRCFG) | BMP581_OSR_CFG_PRESS_EN);

  // Set ground pressure reading on init
  baro->readPress(baro, &baro->groundPress);

  DeviceHandle_t handle;
  strcpy(handle.name, name);
  handle.device = baro;
  return handle;
}

/******************************** DEVICE METHODS ********************************/

/* =============================================================================== */
/**
 * @brief Updates the BMP581 barometer readings.
 * @param *baro Pointer to BMP581 struct to be updated.
 * @returns @c NULL.
 **
 * =============================================================================== */
void BMP581_update(BMP581 *baro) {
  baro->readRawTemp(baro, baro->rawTemp);
  baro->processRawTemp(baro, baro->rawTemp, &baro->temp);

  baro->readRawPress(baro, baro->rawPress);
  baro->processRawPress(baro, baro->rawPress, &baro->press);
}

/* =============================================================================== */
/**
 * @brief Read the temperature from the BMP581 sensor.
 * @param *baro Pointer to BMP581 struct.
 * @param *out Pointer to float where the temperature will be stored.
 * @returns @c NULL.
 **
 * =============================================================================== */
void BMP581_readTemp(BMP581 *baro, float *out) {
  uint8_t bytes[BMP581_DATA_TOTAL];
  baro->readRawTemp(baro, bytes);
  baro->processRawTemp(baro, bytes, out);
}

/* =============================================================================== */
/**
 * @brief Processes raw temperature data from BMP581 sensor.
 * @param *baro  Pointer to BMP581 struct.
 * @param *bytes Pointer to array containing raw temperature.
 * @param *out   Pointer to a float where processed temperature value will be stored.
 * @returns @c NULL.
 **
 * =============================================================================== */
void BMP581_processRawTemp(BMP581 *baro, uint8_t *bytes, float *out) {
  *out = baro->tempSensitivity * (int32_t)(((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | bytes[0]);
}

/* =============================================================================== */
/**
 * @brief
 * @param
 * @param
 * @returns @c NULL.
 **
 * =============================================================================== */
void BMP581_readRawTemp(BMP581 *baro, uint8_t *out) {
  out[0] = BMP581_readRegister(baro, BMP581_TEMPERATURE_MSB);  // temp high
  out[1] = BMP581_readRegister(baro, BMP581_TEMPERATURE_LSB);  // temp low
  out[2] = BMP581_readRegister(baro, BMP581_TEMPERATURE_XLSB); // temp mid
}

/* =============================================================================== */
/**
 * @brief
 * @param
 * @param
 * @returns @c NULL.
 **
 * =============================================================================== */
void BMP581_readPress(BMP581 *baro, float *out) {
  uint8_t bytes[BMP581_DATA_TOTAL];
  baro->readRawPress(baro, bytes);
  baro->processRawPress(baro, bytes, out);
}

/* =============================================================================== */
/**
 * @brief
 * @param
 * @param
 * @returns @c NULL.
 **
 * =============================================================================== */
void BMP581_processRawPress(BMP581 *baro, uint8_t *bytes, float *out) {
  *out = baro->pressSensitivity * (int32_t)(((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | bytes[0]);
}

/* =============================================================================== */
/**
 * @brief
 * @param
 * @param
 * @returns @c NULL.
 **
 * =============================================================================== */
void BMP581_readRawPress(BMP581 *baro, uint8_t *out) {
  out[0] = BMP581_readRegister(baro, BMP581_PRESSURE_MSB);  // temp high
  out[1] = BMP581_readRegister(baro, BMP581_PRESSURE_LSB);  // temp low
  out[2] = BMP581_readRegister(baro, BMP581_PRESSURE_XLSB); // temp mid
}

/******************************** INTERFACE METHODS ********************************/

void BMP581_writeRegister(BMP581 *baro, uint8_t address, uint8_t data) {
  SPI spi = baro->base;

  spi.port->ODR &= ~spi.cs;

  // Send read command and address
  uint8_t payload = address & 0x7F; // Load payload with address and read command
  spi.transmit(&spi, payload);      // Transmit payload
  spi.transmit(&spi, data);         // Transmit dummy data and read response data

  spi.port->ODR |= spi.cs;
}

uint8_t BMP581_readRegister(BMP581 *baro, uint8_t address) {
  uint8_t response = 0;
  SPI spi          = baro->base;

  spi.port->ODR &= ~spi.cs;

  // Send read command and address
  uint8_t payload = address | 0x80;              // Load payload with address and read command
  response        = spi.transmit(&spi, payload); // Transmit payload
  response        = spi.transmit(&spi, 0xFF);    // Transmit dummy data and read response data

  spi.port->ODR |= spi.cs;

  return response;
}
