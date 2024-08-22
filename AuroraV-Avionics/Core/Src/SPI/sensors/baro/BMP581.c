/***********************************************************************************
 * @file        BMP581.c                                                           *
 * @author      Matt Ricci                                                         *
 * @addtogroup  BMP581                                                             *
 *                                                                                 *
 * @todo Add altitude calculation method                                           *
 * @todo Document implementation                                                   *
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
void BMP581_init(BMP581 *baro, GPIO_TypeDef *port, unsigned long cs, float tempSensitivity, float pressSensitivity) {
  SPI_init(&baro->base, SENSOR_BARO, SPI1, port, cs);
  baro->tempSensitivity  = tempSensitivity;
  baro->pressSensitivity = pressSensitivity;
  baro->update           = BMP581_update;
  baro->readTemp         = BMP581_readTemp;
  baro->readRawTemp      = BMP581_readRawTemp;
  baro->processRawTemp   = BMP581_processRawTemp;
  baro->readPress        = BMP581_readPress;
  baro->readRawPress     = BMP581_readRawPress;
  baro->processRawPress  = BMP581_processRawPress;
	
  const uint32_t superDelay = 0xFFFF;
  volatile uint8_t counter = 0;

  // Wait for the spefified period - need to wait for 2ms here.
  for(uint32_t i = 0; i < superDelay; i++) {
	  counter++;
  }

  uint8_t chipID = 0;

  chipID = BMP581_readRegister(baro, 0x01);

  BMP581_writeRegister(baro, BMP581_ODR_CFG, BMP581_ODR_CFG_PWR | BMP581_ODR_CFG_DEEP_DIS);
  uint8_t OSRCFG = BMP581_readRegister(baro, BMP581_OSR_CFG);
  BMP581_writeRegister(baro, BMP581_OSR_CFG, (BMP581_OSR_CFG_RESERVED & OSRCFG) | BMP581_OSR_CFG_PRESS_EN);

  // Set ground pressure reading on init
  baro->readPress(baro, &baro->groundPress);
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
void BMP581_update(BMP581 *baro) {
  baro->readRawTemp(baro, baro->rawTemp);
  baro->processRawTemp(baro, baro->rawTemp, &baro->temp);

  baro->readRawPress(baro, baro->rawPress);
  baro->processRawPress(baro, baro->rawPress, &baro->press);
}

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

   uint8_t response = 0;
   SPI spi = baro->base;

   // Manually drop the chip select.
   GPIOA->ODR &= ~(1 << GPIO_ODR_OD3_Pos);

  // Wait for the SPI bus to become ready.
  while((SPI1->SR & SPI_SR_TXE) == 0);

  // Send out the device address
  SPI1->DR = (address & 0x7F);

  // Wait for the recieve to become available.
  while((SPI1->SR & SPI_SR_RXNE) == 0);

  // Read the dummy response.
  response = SPI1->DR;

  // Send the next byte (data)
  while((SPI1->SR & SPI_SR_TXE) == 0);

  // Send out the device address
  SPI1->DR = data;

  // Wait for the recieve to become available.
  while((SPI1->SR & SPI_SR_RXNE) == 0);

  // Read the dummy response.
  response = SPI1->DR;

  // Wait for the peripheral to finsh.
  while((SPI1->SR & SPI_SR_BSY) == SPI_SR_BSY);

  // Manually raise the chip select.
  GPIOA->ODR |= (1 << GPIO_ODR_OD3_Pos);

}

uint8_t BMP581_readRegister(BMP581 *baro, uint8_t address) {

  uint8_t response = 0;

  // Manually drop the chip select.
  GPIOA->ODR &= ~(1 << GPIO_ODR_OD3_Pos);

 // Wait for the SPI bus to become ready.
 while((SPI1->SR & SPI_SR_TXE) == 0);

 // Send out the device address
 SPI1->DR = (address | 0x80);

 // Wait for the recieve to become available.
 while((SPI1->SR & SPI_SR_RXNE) == 0);

 // Read the dummy response.
 response = SPI1->DR;

 // Send the next byte (data)
 while((SPI1->SR & SPI_SR_TXE) == 0);

 // Send out the device address
 SPI1->DR = 0xFF;

 // Wait for the recieve to become available.
 while((SPI1->SR & SPI_SR_RXNE) == 0);

 // Read the dummy response.
 response = SPI1->DR;

 // Wait for the peripheral to finsh.
 while((SPI1->SR & SPI_SR_BSY) == SPI_SR_BSY);

 // Manually raise the chip select.
 GPIOA->ODR |= (1 << GPIO_ODR_OD3_Pos);

  return response;
}