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

uint8_t read_FLASH(uint8_t address) {
  uint16_t return_value     = 0;
  uint16_t payload_to_send  = 0;
  payload_to_send          |= (address << 0x8); // load address into top 7 bits
  while ((SPI2->SR & SPI_SR_TXE) == 0);
  GPIOE->ODR &= (~(GPIO_ODR_OD11));
  SPI2->DR    = payload_to_send;
  while ((SPI2->SR & SPI_SR_RXNE) == 0);        // wait for received data
  GPIOE->ODR   |= (GPIO_ODR_OD11);
  return_value  = (uint16_t)(SPI2->DR);
  return (uint8_t)return_value;
}

void configure_SPI2_Flash(void) {
  // port E pins 12, 13 and 14 to AF5 for SCLK, SDO, SDI
  GPIOE->MODER  &= ~FLASH_DATA_MODE_Msk;
  GPIOE->MODER  |= FLASH_DATA_MODE_Set(0x02);  // Set alternate function mode
  GPIOE->AFR[1] &= ~FLASH_DATA_AFRH_Msk;       // clears AFRH 12, 13, 14
  GPIOE->AFR[1] |= FLASH_DATA_AFRH_Set(0x05);  // sets AFRH 12, 13, 14 to AF5 for Flash SPI

  GPIOE->OTYPER  &= ~FLASH_DATA_OTYPE_Msk;     // Set push-pull
  GPIOE->OSPEEDR &= ~FLASH_DATA_OSPEED_Msk;    // Clear speed register
  GPIOE->OSPEEDR |= FLASH_DATA_MODE_Set(0x02); // Set slew rate high

  // port E pins 9, 10 and 11 need to be configured as outputs
  GPIOE->MODER   &= ~FLASH_CONTROL_MODE_Msk;
  GPIOE->MODER   |= FLASH_CONTROL_MODE_Set(0x01);
  GPIOE->OTYPER  &= ~FLASH_CONTROL_OTYPE_Msk;      // sets 0xboth as push-pull
  GPIOE->OSPEEDR &= ~FLASH_CONTROL_OSPEED_Msk;     // Clear speed register
  GPIOE->OSPEEDR |= FLASH_CONTROL_SPEED_Set(0x02); // sets slew rate as high speed

  // PE9, high allows status register of Flash to be configured, low default
  // PE10 low pauses device, high default

  GPIOE->ODR |= FLASH_CONTROL_ODR_Msk;

  SPI2->CR1 &= (~(SPI_CR1_CPHA_Msk) | (SPI_CR1_CPOL_Msk)); // sets CPOL and CPHA to zero as specified
                                                           // in LoRa datasheet
  //
  SPI2->CR1 &= (~(SPI_CR1_BR_Msk)); // i think we can leave it as fclck/2 = 42MHz
  // SPI2->CR1 |= (0x2 << SPI_CR1_BR_Pos);// set board rate too fclck / 16 =
  // 42/8 = 5.25 (10 MHz max for LoRa)

  // needs bit DIO and Reset configured to idk what
  SPI2->CR1 |= SPI_CR1_MSTR;              // Micro is master
  SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software management
  SPI2->CR1 &= ~SPI_CR1_LSBFIRST_Msk;     // MSB FIRST
  SPI2->CR1 |= SPI_CR1_DFF;               // 16 bit transfer
  SPI2->CR1 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_RXONLY);
  SPI2->CR1 |= (0x1 << SPI_CR1_SPE_Pos);  // Enable SPI
}
