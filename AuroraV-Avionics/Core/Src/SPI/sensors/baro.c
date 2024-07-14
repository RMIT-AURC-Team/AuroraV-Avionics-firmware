#include "baro.h"

void Baro_init(Sensor_multi *baro, unsigned long cs, int count, ...) {
  baro->cs    = cs;
  baro->write = Baro_write;
  baro->read  = Baro_read;

  // Assign sensitivities from varargs
  va_list args;
  va_start(args, count);
  for (int i = 0; i < count; i++) {
    baro->sensitivities[i] = (float)va_arg(args, double); // float promoted to double in varargs
  }
  va_end(args);

  // Configure barometer
  baro->write(baro, BARO_ODR_CFG, BARO_ODR_CFG_PWR | BARO_ODR_CFG_DEEP_DIS);
  uint8_t OSRCFG = baro->read(baro, BARO_OSR_CFG);
  baro->write(baro, BARO_OSR_CFG, (BARO_OSR_CFG_RESERVED * OSRCFG) | BARO_OSR_CFG_PRESS_EN);
}

void Baro_write(Sensor_multi *baro, uint8_t address, uint8_t payload) {
  uint16_t return_value    = 0;
  uint16_t payload_to_send = 0;
  payload_to_send &= (~(0x8000));        // 0 write, dont need this lne really
  payload_to_send |= (address << 0x8);   // load address into top 7 bits
  payload_to_send |= payload;            // load data in to write
  GPIOA->ODR &= ~baro->cs;               // lower gyro chip select
  while ((SPI1->SR & SPI_SR_TXE) == 0);  // wait for transmission to be empty
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0); // wait for received data
  return_value = (uint16_t)(SPI1->DR);
  while ((SPI1->SR & SPI_SR_BSY) == 1);
  GPIOA->ODR |= baro->cs;
}

uint8_t Baro_read(Sensor_multi *baro, uint8_t address) {
  uint16_t return_value;
  uint16_t payload_to_send = 0;
  payload_to_send |= 0x8000;             // 1 read
  payload_to_send |= (address << 0x8);   // load address into top 7 bits
  GPIOA->ODR &= ~baro->cs;               // lower gyro chip select
  while ((SPI1->SR & SPI_SR_TXE) == 0);  // wait for transmission to be empty
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0); // wait for received data
  return_value = (uint16_t)(SPI1->DR);
  while ((SPI1->SR & SPI_SR_BSY) == 1);
  GPIOA->ODR |= baro->cs;
  return return_value;
}
