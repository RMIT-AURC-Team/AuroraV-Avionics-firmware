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

const struct LoRa_Registers LoRa_Registers = {0, 1, 0xd, 0xE, 0xF, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x28, 0x29, 0x2A, 0x2C, 0x31, 0x33, 0x37, 0x39, 0x3B, 0x3D, 0x40, 0x41};

void configure_SPI3_LoRa() {
  GPIOC->MODER &= (~(GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE12_Msk));
  GPIOC->MODER |= ((0x2 << GPIO_MODER_MODE10_Pos) | (0x2 << GPIO_MODER_MODE11_Pos) | (0x2 << GPIO_MODER_MODE12_Pos));
  GPIOC->PUPDR &= (~(GPIO_PUPDR_PUPD10_Msk | GPIO_PUPDR_PUPD11_Msk | GPIO_PUPDR_PUPD12_Msk));
  GPIOC->PUPDR |= ((0X1 << GPIO_PUPDR_PUPD10_Pos) | (0X1 << GPIO_PUPDR_PUPD11_Pos) | (0X1 << GPIO_PUPDR_PUPD12_Pos));
  GPIOD->PUPDR |= (0X1 << GPIO_PUPDR_PUPD1_Pos);
  GPIOD->MODER &= (~(GPIO_MODER_MODE0_Msk) | (GPIO_MODER_MODE7_Msk) | (GPIO_MODER_MODE1_Msk));
  GPIOD->MODER |= (0X01 << GPIO_MODER_MODE0_Pos) | (0X01 << GPIO_MODER_MODE7_Pos); // chip select stuff
  TIM6->ARR    &= (~(TIM_ARR_ARR_Msk));
  TIM6->PSC    &= (~(TIM_PSC_PSC_Msk));
  TIM6->ARR    |= 20000;
  TIM6->PSC    |= 251;

  GPIOD->ODR |= (GPIO_ODR_OD7);
  TIM6->CR1  |= TIM_CR1_CEN;
  while ((TIM6->SR & TIM_SR_UIF) == 0); // 60 ms delay
  GPIOD->ODR &= (~(GPIO_ODR_OD7));
  TIM6->SR   &= ~(TIM_SR_UIF);          // clears UIF

  GPIOC->OTYPER  &= (~(GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11 | GPIO_OTYPER_OT12));
  GPIOC->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED10_Msk | GPIO_OSPEEDR_OSPEED11_Msk | GPIO_OSPEEDR_OSPEED12_Msk));
  GPIOC->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED10_Pos | 0x2 << GPIO_OSPEEDR_OSPEED11_Pos | 0x2 << GPIO_OSPEEDR_OSPEED12_Pos);
  GPIOD->ODR     |= GPIO_ODR_OD0;                          // raise chip select
  GPIOC->AFR[1]  &= (uint32_t)(~(0x000FFF00));             // clears AFRH 10, 11 and 12
  GPIOC->AFR[1]  |= (0x00066600);                          // sets AFRH 10, 11 and 12 to AF6 for lora SPI

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

void Load_And_Send_To_LoRa(char *pointerdata, const struct LoRa_Registers *LoRa_Registers) {
  write_lora_packet(LoRa_Registers->RegIrqFlags, 0X08);    // clears the status flags
  write_lora_packet(LoRa_Registers->RegFifoAddrPtr, 0X80); // set pointer adddress to TX
  for (int x = 0; x < 16; x++) {
    write_lora_packet(0x00, pointerdata[x]);               // load fifo
  }
  uint8_t regopmode  = 0;
  regopmode          = receive_lora_data(0X1);             // read in current status
  regopmode         &= (unsigned int)(~(0x7));             // clear status bits
  regopmode         |= 0x3;                                // or bits in
  write_lora_packet(0x1, regopmode);                       // set to transmit

                                                           //  while (!(GPIOD->IDR & 0X2));                                  // wait for transmit complete
  write_lora_packet(LoRa_Registers->RegIrqFlags, 0X08); // clears the status flags
  write_lora_packet(0x12, 0X00);
}

void write_lora_packet(uint8_t address, uint8_t payload) {
  uint16_t return_value     = 0;                 // clear varibles
  uint16_t payload_to_send  = 0;
  payload_to_send          &= (~(0XFFFF));       // clear variable
  payload_to_send          |= (address << 0x8);  // load address into top 7 bits
  payload_to_send          |= (1 << 0xF);        // set rw bit to write
  payload_to_send          |= payload;           // mask in data
  GPIOD->ODR               &= (~(GPIO_ODR_OD0)); // lower chip select
  while ((SPI3->SR & SPI_SR_TXE) == 0);          // wait for transmission to be empty
  SPI3->DR = payload_to_send;                    // load data reg
  while ((SPI3->SR & SPI_SR_RXNE) == 0);         // wait for received data
  return_value = (uint16_t)(SPI3->DR);           // see if it returns anything
  while ((SPI3->SR & SPI_SR_BSY) == 1);          // wait for line to be clear
  GPIOD->ODR |= (GPIO_ODR_OD0);                  // raise chip select
}

void configure_LoRa_module() {
  // send_lora_packet( address,  payload)
  //  first read the RegOpMode to see what mode its i
  uint8_t regopmode;
  uint8_t TXPOINTER_ADDRESS;
  uint8_t FIFOPOINTER_ADDRESS;
  /////////////////////////////////////////////////
  write_lora_packet(0x40, 0x40);       // cant remember what this does
  regopmode = receive_lora_data(0X1);  // read in modem statues
  regopmode = 0;                       // set status to sleep
  write_lora_packet(0x1, regopmode);   // put in sleep mode
  regopmode  = receive_lora_data(0X1); // read back status
  regopmode |= 0X80;                   // set relevent buts
  write_lora_packet(0x1, regopmode);   // set into LoRa mode
  regopmode = receive_lora_data(0X1);  //
  write_lora_packet(0x1D, 0x52);       // REG MODEM CONFIG 1// not sure what configuration we want to actually use for 0x1D
                                       ////////////////////////////////////////////////////////// below is the config we intend on using for flight
  write_lora_packet(0x1E, 0x94);       // REG MODEM CONFIG 2
  write_lora_packet(0x22, 0x10);       // REG PAYLOAD LENGTH
  write_lora_packet(0x23, 0x10);       // REG PAYLOAD LENGTH
  //////////////////////////////////////////////////////////////////////// test config for johns lora
  write_lora_packet(0x1D, 0X4A); // cant rememember what this is for
                                 ///////////////////////////////////////// set to standby
  regopmode  = receive_lora_data(0X1);
  regopmode &= (~(0x7));
  regopmode |= 0x1;
  write_lora_packet(0x1, regopmode);
  regopmode = receive_lora_data(0X1);         // SHOULD GET 0X81
                                              /////////////////////////////////////////////////////////
  TXPOINTER_ADDRESS = receive_lora_data(0xE); // should be 80
  write_lora_packet(0XD, 0X80);               // set pointer adddress to TX
}

uint8_t receive_lora_data(uint8_t address) {
  uint16_t return_value;
  uint16_t payload_to_send;
  payload_to_send &= (~(0XFFFF));
  payload_to_send |= (address << 0x8);
  payload_to_send &= (~(1 << 0xF));
  while ((SPI3->SR & SPI_SR_TXE) == 0);
  GPIOD->ODR &= (~(GPIO_ODR_OD0));
  SPI3->DR    = payload_to_send;
  while ((SPI3->SR & SPI_SR_RXNE) == 0); // wait for received data
  GPIOD->ODR   |= (GPIO_ODR_OD0);
  return_value  = (uint16_t)(SPI3->DR);
  return (uint8_t)return_value;
}
