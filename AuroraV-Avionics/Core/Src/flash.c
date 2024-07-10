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

void configure_SPI2_Flash(void) {
  // port E pins 12, 13 and 14 to AF5 for SPI
  GPIOE->MODER  &= (~(GPIO_MODER_MODE12_Msk | GPIO_MODER_MODE13_Msk | GPIO_MODER_MODE14_Msk));
  GPIOE->MODER  |= ((0x2 << GPIO_MODER_MODE12_Pos) | (0x2 << GPIO_MODER_MODE13_Pos) | (0x2 << GPIO_MODER_MODE14_Pos));
  GPIOE->AFR[1] &= (uint32_t)(~(0x0FFF0000)); // clears AFRH 10, 11 and 12
  GPIOE->AFR[1] |= (0x05550000);              // sets AFRH 10, 11 and 12 to AF6 for lora SPI
  // port E pins 11, 10 and 9 need to be configured as outputs

  GPIOE->MODER   &= (~(GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk));
  GPIOE->MODER   |= ((0x1 << GPIO_MODER_MODE9_Pos) | (0x1 << GPIO_MODER_MODE10_Pos) | (0x1 << GPIO_MODER_MODE11_Pos));
  GPIOE->OTYPER  &= (uint16_t)(~(GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11));                                          // sets 0xboth as push-pull
  GPIOE->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED9_Msk | GPIO_OSPEEDR_OSPEED10_Msk | GPIO_OSPEEDR_OSPEED11_Msk));                         // clears Port 14 and 15 section
  GPIOE->OSPEEDR |= ((0x2 << GPIO_OSPEEDR_OSPEED9_Pos) | (0x2 << GPIO_OSPEEDR_OSPEED10_Pos) | (0x2 << GPIO_OSPEEDR_OSPEED11_Pos)); // sets slew rate as high speed

  GPIOE->OTYPER  &= (~(GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT14));
  GPIOE->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED12_Msk | GPIO_OSPEEDR_OSPEED13_Msk | GPIO_OSPEEDR_OSPEED14_Msk));
  GPIOE->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED12_Pos | 0x2 << GPIO_OSPEEDR_OSPEED13_Pos | 0x2 << GPIO_OSPEEDR_OSPEED14_Pos);

  // PE9, high allows status register of Flash to be configured, low default
  // PE10 low pauses device, high default

  GPIOE->ODR |= (GPIO_ODR_OD10) | (GPIO_ODR_OD11) | (GPIO_ODR_OD9);
  SPI2->CR1  &= (~(SPI_CR1_BR_Msk)); // i think we can leave it as fclck/2 = 42MHz

  //		SPI2->CR1 |= (0x2 << SPI_CR1_BR_Pos);// set board rate too fclck / 16 = 42/8 = 5.25 (10 MHz max for LoRa)
  SPI2->CR1 &= (~(SPI_CR1_CPHA_Msk) | (SPI_CR1_CPOL_Msk)); // sets CPOL and CPHA to zero as specified in LoRa datasheet

  // needs bit DIO and Reset configured to idk what
  SPI2->CR1 |= SPI_CR1_MSTR;              // micro is master
  SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software management
  SPI2->CR1 &= (~(SPI_CR1_LSBFIRST_Msk)); // MSB FIRST
  SPI2->CR1 |= SPI_CR1_DFF;               // 16 bit transfer
  SPI2->CR1 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_RXONLY);
  SPI2->CR1 |= (0x1 << SPI_CR1_SPE_Pos);
}

uint8_t read_FLASH_status(uint8_t address) {
  uint8_t return_value;
  while ((SPI2->SR & SPI_SR_TXE) == 0);
  GPIOE->ODR &= (~(GPIO_ODR_OD11));
  SPI2->DR    = address;
  while ((SPI2->SR & SPI_SR_RXNE) == 0); // wait for received data
  GPIOE->ODR   |= (GPIO_ODR_OD11);
  return_value  = (uint8_t)(SPI2->DR);
  return (uint8_t)return_value;
}

void write_FLASH_status(uint8_t address, uint8_t data) {
  // uint8_t return_value;
  while ((SPI2->SR & SPI_SR_TXE) == 0);
  GPIOE->ODR &= (~(GPIO_ODR_OD11));
  SPI2->DR    = address;
  while ((SPI2->SR & SPI_SR_BSY));
  SPI2->DR = data;
  while ((SPI2->SR & SPI_SR_BSY));
  GPIOE->ODR |= (GPIO_ODR_OD11);
  // eturn_value = (uint8_t)(SPI2->DR);
}
void Flash_Page_Program(unsigned int address, uint8_t *pointer, uint8_t number_of_bytes) // exact same as gyro
{
  uint8_t byte2, byte3, byte4;
  unsigned int temp = (address & 0xFFFFFF) >> 16;
  byte2             = (uint8_t)temp;
  temp              = (address & 0xFFFF) >> 8;
  byte3             = (uint8_t)temp;
  temp              = (address & 0xFF);
  byte4             = (uint8_t)temp;
  Flash_Write_Enable();
  GPIOE->ODR &= (~(GPIO_ODR_OD11));     // lower gyro chip select
  while ((SPI2->SR & SPI_SR_TXE) == 0); // wait for transmission to be empty
  SPI2->DR = 0x02;
  while ((SPI2->SR & SPI_SR_BSY));
  SPI2->DR = byte2;
  while ((SPI2->SR & SPI_SR_BSY));
  SPI2->DR = byte3;
  while ((SPI2->SR & SPI_SR_BSY));
  SPI2->DR = byte4;
  while ((SPI2->SR & SPI_SR_BSY));
  for (uint8_t x = 0; x < number_of_bytes; x++) {
    SPI2->DR = pointer[x];
    while ((SPI2->SR & SPI_SR_BSY));
  }
  GPIOE->ODR |= (GPIO_ODR_OD11);
}
void Flash_Chip_Erase() {
  GPIOE->ODR &= (~(GPIO_ODR_OD11));     // lower gyro chip select
  while ((SPI2->SR & SPI_SR_TXE) == 0); // wait for transmission to be empty
  SPI2->DR = 0x60;
  while ((SPI2->SR & SPI_SR_BSY));
  GPIOE->ODR |= (GPIO_ODR_OD11);
}
void Flash_Write_Enable() {
  GPIOE->ODR &= (~(GPIO_ODR_OD11));     // lower gyro chip select
  while ((SPI2->SR & SPI_SR_TXE) == 0); // wait for transmission to be empty
  SPI2->DR = 6;
  while ((SPI2->SR & SPI_SR_BSY));
  GPIOE->ODR |= (GPIO_ODR_OD11);
}
