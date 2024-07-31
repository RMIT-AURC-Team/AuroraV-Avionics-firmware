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
void BMP581_init(BMP581 *baro, GPIO_TypeDef *port, unsigned long cs, float tempSensitivity, float pressSensitivity) {
  SPI_init(&baro->base, SENSOR_BARO, SPI1, port, cs);
  baro->tempSensitivity  = tempSensitivity;
  baro->pressSensitivity = pressSensitivity;
  baro->readTemp         = BMP581_readTemp;
  baro->readRawTemp      = BMP581_readRawTemp;
  baro->processRawTemp   = BMP581_processRawTemp;
  baro->readPress        = BMP581_readPress;
  baro->readRawPress     = BMP581_readRawPress;
  baro->processRawPress  = BMP581_processRawPress;

  BMP581_writeRegister(baro, BMP581_ODR_CFG, BMP581_ODR_CFG_PWR | BMP581_ODR_CFG_DEEP_DIS);
  uint8_t OSRCFG = BMP581_readRegister(baro, BMP581_OSR_CFG);
  BMP581_writeRegister(baro, BMP581_OSR_CFG, (BMP581_OSR_CFG_RESERVED & OSRCFG) | BMP581_OSR_CFG_PRESS_EN);
}

/******************************** DEVICE METHODS ********************************/

/* =============================================================================== */
/**
 * @brief 
 * @param 	
 * @param 	
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
 * @brief 
 * @param 	
 * @param 
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
  uint16_t response;
  SPI spi = baro->base;

  // Send write command with address and data
  uint16_t payload = (address << 0x08) | data;
  spi.port->ODR &= ~spi.cs;
  spi.send(&spi, payload);

  // Read in response from interface
  spi.receive(&spi, &response);
  spi.port->ODR |= spi.cs;
}

uint8_t BMP581_readRegister(BMP581 *baro, uint8_t address) {
  uint16_t response;
  SPI spi = baro->base;

  // Send write command with address and data
  uint16_t payload = (address << 0x8) | 0x8000;
  spi.port->ODR &= ~spi.cs;
  spi.send(&spi, payload);

  // Read in response from interface
  spi.receive(&spi, &response);
  spi.port->ODR |= spi.cs;
  return (uint8_t)response;
}
