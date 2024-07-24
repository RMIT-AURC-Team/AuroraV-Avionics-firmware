#include "flash.h"

/* SPI2 FLASH
 * ---------------------------------------------
 * Flash Pin       | MCU GPIO Pin  | SIGNAL TYPE
 * ----------------|---------------|------------
 * SDI             | PE14          | DATA
 * SDO             | PE13          | DATA
 * SCLK            | PE12          | DATA
 * CS              | PE11          | CONTROL
 * Memory Hold     | PE10          | CONTROL
 * Write Protect   | PE9           | CONTROL    */

void Flash_init(Flash *flash, GPIO_TypeDef *port, unsigned long cs) {
  flash->base.device = MEMORY_FLASH;
  flash->base.port   = port;
  flash->base.cs     = cs;
  flash->erase       = Flash_erase;
  flash->read        = Flash_read;
  flash->write       = Flash_write;
}

/******************************** PRIVATE METHODS ********************************/

void _Flash_writeEnable() {
  unsigned int status_return = 0;
  while ((SPI4->SR & SPI_SR_TXE) == 0);
  GPIOE->ODR &= (~(GPIO_ODR_OD11));
  SPI4->DR = (0x06);
  while ((SPI4->SR & SPI_SR_RXNE) == 0);
  status_return = (SPI4->DR);
  while ((SPI4->SR & SPI_SR_BSY));               // wait for received data
  GPIOE->ODR |= (GPIO_ODR_OD11);
  while ((_Flash_readStatus1() & 0x01) == 0x01); // Wait for busy to clear
}

uint8_t _Flash_readStatus1(void) {
  GPIOE->ODR &= ~(GPIO_ODR_OD11);
  SPI4->DR = (0x05);
  while ((SPI4->SR & SPI_SR_RXNE) == 0);
  uint8_t status_return = (uint8_t)(SPI4->DR);
  while ((SPI4->SR & SPI_SR_BSY));
  GPIOE->ODR |= (GPIO_ODR_OD11);
  return status_return;
}

uint8_t _Flash_readStatus2(void) {
  uint8_t status_return;
  uint16_t confirm_data_pass;
  confirm_data_pass = 0;
  confirm_data_pass = (0x5);
  GPIOE->ODR &= (~(GPIO_ODR_OD11));
  SPI4->DR = (0x35);
  while ((SPI4->SR & SPI_SR_RXNE) == 0);
  status_return = (uint8_t)(SPI4->DR);
  while ((SPI4->SR & SPI_SR_BSY));
  GPIOE->ODR |= (GPIO_ODR_OD11);
  //	SPI4->CR1 |= (SPI_CR1_DFF);
  return status_return;
}

uint8_t _Flash_readStatus3(void) {
  uint8_t status_return;
  uint16_t confirm_data_pass;
  confirm_data_pass = 0;
  confirm_data_pass = (0x5);
  GPIOE->ODR &= (~(GPIO_ODR_OD11));
  SPI4->DR = (0x15);
  while ((SPI4->SR & SPI_SR_RXNE) == 0);
  status_return = (uint8_t)(SPI4->DR);
  while ((SPI4->SR & SPI_SR_BSY));
  GPIOE->ODR |= (GPIO_ODR_OD11);
  return status_return;
}

/******************************** DEVICE METHODS ********************************/

void Flash_erase(void) {
  _Flash_writeEnable();
  unsigned int status_return = 0;
  SPI4->CR1 &= (~(SPI_CR1_DFF));
  uint32_t return_value = (0X06);
  while ((SPI4->SR & SPI_SR_TXE) == 0);
  GPIOE->ODR &= (~(GPIO_ODR_OD11));
  SPI4->DR = (0XC7);
  while ((SPI4->SR & SPI_SR_TXE) == 0);
  while ((SPI4->SR & SPI_SR_RXNE) == 0);
  return_value = (uint8_t)(SPI4->DR);
  while ((SPI4->SR & SPI_SR_BSY)); // wait for received data
  GPIOE->ODR |= (GPIO_ODR_OD11);
  do {
    while ((SPI4->SR & SPI_SR_TXE) == 0);
    GPIOE->ODR &= (~(GPIO_ODR_OD11));
    SPI4->DR = (0X5);
    while ((SPI4->SR & SPI_SR_RXNE) == 0);
    status_return = (SPI4->DR);
    while ((SPI4->SR & SPI_SR_TXE) == 0);
    SPI4->DR = (0X0F);
    while ((SPI4->SR & SPI_SR_RXNE) == 0);
    status_return = (SPI4->DR);
    while ((SPI4->SR & SPI_SR_BSY)); // wait for received data
    GPIOE->ODR |= (GPIO_ODR_OD11);
    status_return &= 0x1;
  } while (status_return & 0x1);
}

uint8_t Flash_write(Flash *flash, uint32_t address, uint8_t *data) {
  _Flash_writeEnable();
  uint16_t return_value = 0;
  while ((SPI4->SR & SPI_SR_TXE) == 0);
  GPIOE->ODR &= (~(GPIO_ODR_OD11));
  SPI4->DR = 0x2;
  while ((SPI4->SR & SPI_SR_RXNE) == 0); // wait for received data
  return_value = (uint8_t)(SPI4->DR);

  // sent to flash
  for (int x = 0; x < 255; x++) {
    while ((SPI4->SR & SPI_SR_TXE) == 0);
    SPI4->DR = data[x];
    while ((SPI4->SR & SPI_SR_RXNE) == 0); // wait for received data
    return_value = (uint8_t)(SPI4->DR);
  }
  while ((SPI4->SR & SPI_SR_BSY));         // wait for received data
  GPIOE->ODR |= (GPIO_ODR_OD11);

  return (uint8_t)return_value;
}

uint8_t Flash_read(Flash *flash, uint32_t address, uint8_t *data) {
  _Flash_writeEnable();
  uint32_t status_return = 0;
  do {
    while ((SPI4->SR & SPI_SR_TXE) == 0);
    GPIOE->ODR &= (~(GPIO_ODR_OD11));
    SPI4->DR = (0X5);
    while ((SPI4->SR & SPI_SR_RXNE) == 0);

    status_return = (SPI4->DR);
    while ((SPI4->SR & SPI_SR_TXE) == 0);
    SPI4->DR = (0X0F);
    while ((SPI4->SR & SPI_SR_RXNE) == 0);
    status_return = (SPI4->DR);
    while ((SPI4->SR & SPI_SR_BSY)); // wait for received data
    GPIOE->ODR |= (GPIO_ODR_OD11);

  } while (status_return & 0x1);

  uint16_t return_value = 0;
  char b[256];
  while ((SPI4->SR & SPI_SR_TXE) == 0);
  GPIOE->ODR &= (~(GPIO_ODR_OD11));
  SPI4->DR = 0x3;
  while ((SPI4->SR & SPI_SR_RXNE) == 0); // wait for received data
  b[0] = (uint8_t)(SPI4->DR);

  while ((SPI4->SR & SPI_SR_TXE) == 0);
  SPI4->DR = ((address & 0xFF0000) >> 16);
  while ((SPI4->SR & SPI_SR_RXNE) == 0); // wait for received data
  b[0] = (uint8_t)(SPI4->DR);

  while ((SPI4->SR & SPI_SR_TXE) == 0);
  SPI4->DR = ((address & 0xFF00) >> 8);
  while ((SPI4->SR & SPI_SR_RXNE) == 0); // wait for received data
  b[0] = (uint8_t)(SPI4->DR);

  while ((SPI4->SR & SPI_SR_TXE) == 0);
  SPI4->DR = ((address & 0xFF));
  while ((SPI4->SR & SPI_SR_RXNE) == 0); // wait for received data
  b[0] = (uint8_t)(SPI4->DR);

  // sent to flash
  for (int x = 0; x < 255; x++) {
    while ((SPI4->SR & SPI_SR_TXE) == 0);
    SPI4->DR = (uint8_t)0x1;
    while ((SPI4->SR & SPI_SR_RXNE) == 0); // wait for received data
    data[x] = (uint8_t)(SPI4->DR);
  }
  while ((SPI4->SR & SPI_SR_BSY));         // wait for received data
  GPIOE->ODR |= (GPIO_ODR_OD11);

  return (uint8_t)return_value;
}
