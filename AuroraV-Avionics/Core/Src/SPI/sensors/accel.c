#include "accel.h"

void Accel_init(Sensor *accel, unsigned long cs, int scale) {
  accel->cs    = cs;
  accel->write = Accel_write;
  accel->read  = Accel_read;

  uint8_t GSEL = 0x00;
  if (scale == 32) {
    GSEL               = ACCEL_CNTL1_GSEL(32);
    accel->sensitivity = ACCEL_SENSITIVITY(32);
  } else if (scale == 16) {
    GSEL               = ACCEL_CNTL1_GSEL(16);
    accel->sensitivity = ACCEL_SENSITIVITY(16);
  }

  // Configure accelerometer registers
  accel->write(accel, ACCEL_CNTL1, ACCEL_CNTL1_RES | GSEL);                   // Accel select, selected sensitivity
  uint8_t ODCNTL = accel->read(accel, ACCEL_ODCNTL);                          // Read from register for reserve mask
  accel->write(accel, ACCEL_ODCNTL, (ACCEL_ODCNTL_RESERVED & ODCNTL) | 0x2A); // No filter, fast startup, 800Hz
  accel->write(accel, ACCEL_CNTL1, 0xD0);                                     // Enable PC1
}

void Accel_write(Sensor *accel, uint8_t address, uint8_t payload) {
  uint16_t return_value;
  uint16_t payload_to_send = 0;
  payload_to_send &= ~0x8000;            // 0 write, dont need this lne really
  payload_to_send |= (address << 0x8);   // load address into top 7 bits
  payload_to_send |= payload;            // load data in to write
  while ((SPI1->SR & SPI_SR_TXE) == 0);  // wait for transmission to be empty
  GPIOA->ODR &= ~accel->cs;              // lower chip select
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0); // wait for received data
  return_value = (uint16_t)(SPI1->DR);
  while ((SPI1->SR & SPI_SR_BSY) == 1);
  GPIOA->ODR |= accel->cs;
}

uint8_t Accel_read(Sensor *accel, uint8_t address) {
  uint16_t return_value    = 0;
  uint16_t payload_to_send = 0;
  payload_to_send |= 0x8000;
  payload_to_send |= (address << 0x8);   // load address into top 7 bits
  while ((SPI1->SR & SPI_SR_TXE) == 0);
  GPIOB->ODR &= ~accel->cs;
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0); // wait for received data
  GPIOB->ODR |= accel->cs;
  return_value = (uint16_t)(SPI1->DR);
  return (uint8_t)return_value;
}
