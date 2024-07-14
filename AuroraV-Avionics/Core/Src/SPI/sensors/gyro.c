#include "gyro.h"

void Gyro_init(Sensor *gyro, unsigned long cs, float sensitivity) {
  gyro->cs          = cs;
  gyro->sensitivity = sensitivity;
  gyro->write       = Gyro_write;
  gyro->read        = Gyro_read;
  // Configure gyroscope
  gyro->write(gyro, GYRO_CTRL_REG1, GYRO_CTRL_REG1_ODR_800Hz | GYRO_CTRL_REG1_AXIS_ENABLE | GYRO_CTRL_REG1_PD_ENABLE);
}

void Gyro_write(Sensor *gyro, uint8_t address, uint8_t payload) {
  uint16_t return_value;
  uint16_t payload_to_send = 0;
  payload_to_send &= (~(0X4000));        // clear  multiple transfer bit
  payload_to_send &= (~(0x8000));        // set 15th bit to 0 for write
  payload_to_send |= (address << 0x8);   // load address into top 7 bits
  payload_to_send |= (payload);          // mask in data
  GPIOA->ODR &= ~gyro->cs;               // lower gyro chip select
  while ((SPI1->SR & SPI_SR_TXE) == 0);  // wait for transmission to be empty
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0); // wait for received data
  return_value = (uint16_t)(SPI1->DR);
  while ((SPI1->SR & SPI_SR_BSY) == 1);
  GPIOA->ODR |= gyro->cs;
}

uint8_t Gyro_read(Sensor *gyro, uint8_t address) {
  uint16_t return_value;
  uint16_t payload_to_send = 0;
  payload_to_send &= (~(0x4000));        // clear  multiple transfer bit
  payload_to_send |= 0x8000;
  payload_to_send |= (address << 0x8);   // load address into top 7 bits
  while ((SPI1->SR & SPI_SR_TXE) == 0);
  GPIOA->ODR &= ~gyro->cs;
  SPI1->DR = payload_to_send;
  while ((SPI1->SR & SPI_SR_RXNE) == 0); // wait for received data
  GPIOA->ODR |= gyro->cs;
  return_value = (uint16_t)(SPI1->DR);
  return (uint8_t)return_value;
}
