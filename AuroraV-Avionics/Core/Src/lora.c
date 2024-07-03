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

void write_lora_packet(uint8_t address, uint8_t payload) {
  uint16_t return_value     = 0;                     // clear varibles
  uint16_t payload_to_send  = 0;
  payload_to_send          &= (~(0xFFFF));           // clear variable
  payload_to_send          |= (address << 0x8);      // load address into top 7 bits
  payload_to_send          |= (1 << 0xF);            // set rw bit to write
  payload_to_send          |= payload;               // mask in data
  GPIOD->ODR               &= (~(GPIO_MODER_MODE0)); // lower chip select
  while ((SPI3->SR & SPI_SR_TXE) == 0);              // wait for transmission to be empty
  SPI3->DR = payload_to_send;                        // load data reg
  while ((SPI3->SR & SPI_SR_RXNE) == 0);             // wait for received data
  return_value = (uint16_t)(SPI3->DR);               // see if it returns anything
  while ((SPI3->SR & SPI_SR_BSY) == 1);              // wait for line to be clear
  GPIOD->ODR |= (GPIO_MODER_MODE0);                  // raise chip select
}

void configure_LoRa_module() {
  // send_lora_packet( address,  payload)
  //  first read the RegOpMode to see what mode its i
  uint8_t regopmode;
  uint8_t TXPOINTER_ADDRESS;
  uint8_t FIFOPOINTER_ADDRESS;

  write_lora_packet(LoRa_Registers.RegDioMapping1, 0x40);   // set the TX complete interrupt pin for the LoRa chip
  regopmode = receive_lora_data(LoRa_Registers.RegOpMode);  // read in modem statues
  regopmode = 0;                                            // set status to sleep
  write_lora_packet(LoRa_Registers.RegOpMode, regopmode);   // put in sleep mode
  regopmode  = receive_lora_data(LoRa_Registers.RegOpMode); // read back status
  regopmode |= 0x80;                                        // set relevent buts
  write_lora_packet(LoRa_Registers.RegOpMode, regopmode);   // set into LoRa mode
  regopmode = receive_lora_data(LoRa_Registers.RegOpMode);  //
  write_lora_packet(LoRa_Registers.RegModemConfig1, 0x52);  // REG MODEM CONFIG 1// not sure what configuration we want to actually use for 0x1D

  // Below is the config we intend on using for flight
  write_lora_packet(LoRa_Registers.RegModemConfig2, 0x94);     // REG MODEM CONFIG 2
  write_lora_packet(LoRa_Registers.RegPayloadLength, 0x2);     // REG PAYLOAD LENGTH
  write_lora_packet(LoRa_Registers.RegMaxPayloadLength, 0x10); // REG PAYLOAD LENGTH

  // Test config for johns lora
  write_lora_packet(LoRa_Registers.RegModemConfig1, 0x4A);                 // setting the coding rate, BW and CRC
                                                                           // set to standby
  regopmode  = receive_lora_data(LoRa_Registers.RegOpMode);
  regopmode &= (~(0x7));
  regopmode |= 0x1;
  write_lora_packet(LoRa_Registers.RegOpMode, regopmode);
  regopmode         = receive_lora_data(LoRa_Registers.RegOpMode);         // SHOULD GET 0x81
  TXPOINTER_ADDRESS = receive_lora_data(LoRa_Registers.RegFifoTxBaseAddr); // should be 80
  write_lora_packet(LoRa_Registers.RegFifoAddrPtr, 0x80);                  // set pointer adddress to TX

  // this while loop transmits 'boob' on repeat
  while (1) {
    write_lora_packet(0x00, 0xB0);       // load fifo
    write_lora_packet(0x00, 0x0B);       // load fifo
    regopmode  = receive_lora_data(0x1); // read in current status
    regopmode &= (~(0x7));               // clear status bits
    regopmode |= 0x3;                    // or bits in
    write_lora_packet(0x1, regopmode);   // set to transmit
    while (!(GPIOD->IDR & 0x2));         // wait for transmit complete
    write_lora_packet(0x12, 0x00);       // clears the status flags
    write_lora_packet(0xD, 0x80);        // sets pointer back to 90
    FIFOPOINTER_ADDRESS = receive_lora_data(0xD);
  }
  return;
}

void Load_And_Send_To_LoRa(char *pointerdata) {
  write_lora_packet(LoRa_Registers.RegFifoTxBaseAddr, 0X80);   // set pointer adddress to TX
  for (int x = 0; x < 16; x++) {
    write_lora_packet(LoRa_Registers.RegFifo, pointerdata[x]); // load fifo
  }
  uint8_t regopmode  = 0;
  regopmode          = receive_lora_data(0X1);                 // read in current status
  regopmode         &= (unsigned int)(~(0x7));                 // clear status bits
  regopmode         |= 0x3;                                    // or bits in
  write_lora_packet(LoRa_Registers.RegOpMode, regopmode);      // set to transmit
  while (!(GPIOD->IDR & 0X2));                                 // wait for transmit complete
  write_lora_packet(LoRa_Registers.RegIrqFlags, 0X00);         // clears the status flags
}

uint8_t receive_lora_data(uint8_t address) {
  uint16_t return_value;
  uint16_t payload_to_send;
  payload_to_send &= (~(0xFFFF));
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
