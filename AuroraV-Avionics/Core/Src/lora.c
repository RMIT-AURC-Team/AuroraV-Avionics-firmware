#include "lora.h"

/* SPI1 LORA
 * ------------------------------------
 * Flash Pin  | MCU GPIO Pin  | SIGNAL TYPE
 * -----------|---------------|------------
 * SDI        | PC12          | DATA
 * SDO        | PC11          | DATA
 * SCLK       | PC10          | DATA
 * RST        | PD7           | CONTROL
 * DI0        | PD1           | DATA
 * CS         | PD0           | CONTROL      */

void configure_SPI3_LoRa(void) {
  // Set mode and PUPD for PortC data pins
  GPIOC->MODER &= ~LORA_DATAC_MODE_Msk;
  GPIOC->MODER |= LORA_DATAC_MODE_Set(0x02);
  GPIOC->PUPDR &= ~LORA_DATAC_PUPD_Msk;
  GPIOC->PUPDR |= LORA_DATAC_PUPD_Set(0x01);

  // Set mode and PUPD for PortD data and control pins
  GPIOD->PUPDR |= (0x1 << GPIO_PUPDR_PUPD1_Pos);
  GPIOD->MODER &= ~(LORA_DATAD_MODE_Msk | LORA_CONTROLD_MODE_Msk); // Clear PD0, PD1, PD7
  GPIOD->MODER |= LORA_CONTROLD_MODE_Set(0x01);                    // Chip select stuff
  // ^ Unsure on what exactly DI0 (PD1) is used for?
  // seems to be a data input of some kind

  GPIOD->ODR |= (GPIO_ODR_OD7);
  __ASM("NOP");
  __ASM("NOP");
  __ASM("NOP");
  GPIOD->ODR &= (~(GPIO_ODR_OD7));

  GPIOC->OTYPER  &= ~LORA_DATAC_OTYPE_Msk;
  GPIOC->OSPEEDR &= ~LORA_DATAC_OSPEED_Msk;
  GPIOC->OSPEEDR |= LORA_DATAC_SPEED_Set(0x02);

  GPIOD->ODR    |= GPIO_ODR_OD0;                           // raise chip select
  GPIOC->AFR[1] &= ~LORA_DATAC_AFRH_Msk;                   // clears AFRH 10, 11 and 12
  GPIOC->AFR[1] |= LORA_DATAC_AFRH_Set(0x06);              // sets AFRH 10, 11 and 12 to AF6 for lora SPI

  SPI3->CR1 &= (~(SPI_CR1_BR_Msk));
  SPI3->CR1 |= (0x2 << SPI_CR1_BR_Pos);                    // set board rate too fclck / 16 = 42/8 = 5.25 (10 MHz max for LoRa)
  SPI3->CR1 &= (~(SPI_CR1_CPHA_Msk) | (SPI_CR1_CPOL_Msk)); // sets CPOL and CPHA to zero as specified in LoRa datasheet

  // needs bit DIO and Reset configured to idk what
  SPI3->CR1 |= SPI_CR1_MSTR;              // micro is master
  SPI3->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software management
  SPI3->CR1 &= (~(SPI_CR1_LSBFIRST_Msk)); // MSB FIRST
  SPI3->CR1 |= SPI_CR1_DFF;
  SPI3->CR1 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_RXONLY);
  SPI3->CR1 |= (0x1 << SPI_CR1_SPE_Pos);
}
