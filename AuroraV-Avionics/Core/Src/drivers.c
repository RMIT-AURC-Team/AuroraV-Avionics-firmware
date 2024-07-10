#include "drivers.h"

// ===============================================================
//                       RCC INITIALISATION
// ===============================================================

// configure RCC APB1 : CAN1, CAN2, UART3, SPI2, SPI3, TIM6, TIM7,
void configure_RCC_APB1(void) {
  RCC->APB1ENR  |= (RCC_APB1ENR_CAN1EN | RCC_APB1ENR_CAN2EN | RCC_APB1ENR_USART3EN | RCC_APB1ENR_SPI3EN | RCC_APB1ENR_SPI2EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN);
  RCC->APB1RSTR |= (RCC_APB1RSTR_CAN1RST | RCC_APB1RSTR_CAN2RST | RCC_APB1RSTR_USART3RST | RCC_APB1RSTR_SPI3RST | RCC_APB1RSTR_SPI2RST | RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST);
  __ASM("NOP");
  __ASM("NOP");
  RCC->APB1RSTR &= (uint16_t)(~(RCC_APB1RSTR_CAN1RST | RCC_APB1RSTR_CAN2RST | RCC_APB1RSTR_USART3RST | RCC_APB1RSTR_SPI3RST | RCC_APB1RSTR_SPI2RST | RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST));
  __ASM("NOP");
  __ASM("NOP");
}

// configure RCC APB2 : SPI1, USART6
void configure_RCC_APB2(void) {
  RCC->APB2ENR  |= (RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART6EN);
  RCC->APB2RSTR |= (RCC_APB2RSTR_SPI1RST | RCC_APB2RSTR_USART6RST);
  __ASM("NOP");
  __ASM("NOP");
  RCC->APB2RSTR &= (uint16_t)(~(RCC_APB2RSTR_SPI1RST | RCC_APB2RSTR_USART6RST));
  __ASM("NOP");
  __ASM("NOP");
}

// configure RCC AHB1 GPIO A, B, C, D, E
void configure_RCC_AHB1(void) {
  RCC->AHB1ENR  |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN);
  RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOBRST | RCC_AHB1RSTR_GPIOCRST | RCC_AHB1RSTR_GPIODRST | RCC_AHB1RSTR_GPIOERST);
  __ASM("NOP");
  __ASM("NOP");
  RCC->AHB1RSTR &= (uint16_t)(~(RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOBRST | RCC_AHB1RSTR_GPIOCRST | RCC_AHB1RSTR_GPIODRST | RCC_AHB1RSTR_GPIOERST));
}

// ===============================================================
//                         SPI PERIPHERALS
// ===============================================================

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

void configure_SPI3_LoRa(void) {
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

void configure_SPI1_Sensor_Suite(void) {
  GPIOA->MODER  &= (~(GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk));
  GPIOA->MODER  |= ((0x2 << GPIO_MODER_MODE5_Pos) | (0x2 << GPIO_MODER_MODE6_Pos) | (0x2 << GPIO_MODER_MODE7_Pos));
  GPIOA->AFR[0] &= (~(0xFFF00000));                                                                                        // clears AFRL 5, 6 and 7
  GPIOA->AFR[0] |= (0x55500000);                                                                                           // sets ports 5,6,7 to AF5

  GPIOA->OTYPER  &= (~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7));                                              // configure as push pull
  GPIOA->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED5_Msk | GPIO_OSPEEDR_OSPEED6_Msk | GPIO_OSPEEDR_OSPEED7_Msk));                   // clears OSPEED
  GPIOA->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED5_Pos | 0x2 << GPIO_OSPEEDR_OSPEED6_Pos | 0x2 << GPIO_OSPEEDR_OSPEED7_Pos); // sets as high speed

  SPI1->CR1 |= SPI_CR1_MSTR;
  //	SPI1->CR1 |= SPI_CR1_LSBFIRST;
  SPI1->CR1 |= SPI_CR1_DFF;                 // 16 bit transfer
  SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
  SPI1->CR1 &= (~(SPI_CR1_BR_Msk));
  SPI1->CR1 |= (0x3 << SPI_CR1_BR_Pos);     // FOR 5.25 mhz SCK
  SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA; // high sck idle

  // pins PC2, PC3, PC4 need to be set to interrupt pins
  //
  //  pins PA2, PA4, PA1, PB0 need to set as general purpose outputs
  GPIOA->MODER   &= (~(GPIO_MODER_MODE1_Msk | GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk | GPIO_MODER_MODE4_Msk));
  GPIOA->MODER   |= ((0x1 << GPIO_MODER_MODE1_Pos) | (0x1 << GPIO_MODER_MODE2_Pos) | (0x1 << GPIO_MODER_MODE3_Pos) | (0x1 << GPIO_MODER_MODE4_Pos));
  GPIOA->OTYPER  &= (uint32_t)(~(GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3 | GPIO_OTYPER_OT4));                                                            // sets 0xboth as push-pull
  GPIOA->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED1_Msk | GPIO_OSPEEDR_OSPEED2_Msk | GPIO_OSPEEDR_OSPEED3_Msk | GPIO_OSPEEDR_OSPEED4_Msk));                                  // clears Port 14 and 15 section
  GPIOA->OSPEEDR |= ((0x2 << GPIO_OSPEEDR_OSPEED1_Pos) | (0x2 << GPIO_OSPEEDR_OSPEED2_Pos) | (0x2 << GPIO_OSPEEDR_OSPEED3_Pos) | (0x2 << GPIO_OSPEEDR_OSPEED4_Pos)); // sets slew rate as high speed
  // PA2 Gryo Chip Select - set to high for default
  //  PA1 Accel 1 Chip Select - set to high for default
  // PA4 Mag - chip select - set to high for dafault

  GPIOA->ODR |= (GPIO_ODR_OD1 | GPIO_ODR_OD2 | GPIO_ODR_OD3 | GPIO_ODR_OD4); // sets to high to disable chip select

  GPIOB->MODER   &= (~(GPIO_MODER_MODE0_Msk));
  GPIOB->MODER   |= (0x1 << GPIO_MODER_MODE0_Pos);
  GPIOB->OTYPER  &= (~(GPIO_OTYPER_OT0));
  GPIOB->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED0_Msk));
  GPIOB->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED0_Pos);
  // PB0 Accel 1 Chip Select - set to high for default
  GPIOB->ODR |= (GPIO_ODR_OD0);
  SPI1->CR1  |= SPI_CR1_SPE;
}

// ===============================================================
//                       UART AND GPIO
// ===============================================================

void configure_UART3_GPS(void) {
  GPIOD->MODER  &= (~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE13_Msk));
  GPIOD->MODER  |= ((0x2 << GPIO_MODER_MODE8_Pos) | (0x2 << GPIO_MODER_MODE9_Pos) | (0x1 << GPIO_MODER_MODE13_Pos));
  GPIOD->AFR[1] &= (uint32_t)(~(0x000000FF)); // clears AFRL 6 and 7
  GPIOD->AFR[1] |= (0x00000077);              // sets PD 8, 9 and 13 to AF7

  GPIOD->PUPDR |= (0X1 << GPIO_PUPDR_PUPD9_Pos);

  GPIOD->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9 | GPIO_OSPEEDR_OSPEED13));
  GPIOD->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED8_Pos) | (0x3 << GPIO_OSPEEDR_OSPEED9_Pos) | (0x3 << GPIO_OSPEEDR_OSPEED13_Pos);

  // need over sampling = 1
  USART3->BRR &= (unsigned int)(0xFFFF0000); //  clear mantissa and div in baud rate reg
  USART3->BRR |= (0x0002227);                // set mantissa and div in baud rate reg to 9600

  USART3->CR1 &= (unsigned int)(~(0x400));   // disable parity
  USART3->CR2 &= (unsigned int)(~(0xE00));   // disable synchrnous mode
  USART3->CR3 &= (unsigned int)(~(0x300));   // disable flow control
  USART3->CR1 |= (unsigned int)(0x200C);     // enable usart, enable receive and transmitt
  USART3->CR1 |= USART_CR1_OVER8;

  // turn reset pin high
  GPIOD->ODR |= GPIO_ODR_OD13;
}

void configure_UART6_Serial_2_mini_USB(void) {
  GPIOC->MODER  &= (~(GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk | GPIO_MODER_MODE8_Msk));
  GPIOC->MODER  |= ((0x2 << GPIO_MODER_MODE6_Pos) | (0x2 << GPIO_MODER_MODE7_Pos) | (0x2 << GPIO_MODER_MODE8_Pos));
  GPIOC->AFR[0] &= (~(0xFF000000));           // clears AFRL 6 and 7 and
  GPIOC->AFR[1] &= (uint32_t)(~(0x0000000F)); // clears AFRH 8
  GPIOC->AFR[0] |= (0x88000000);              // sets PC 6 and 7 to AF8
  GPIOC->AFR[1] |= (0x00000008);              // sets PC 8 to AF8
}

// General GPIO Configure for MISC: LED1, LED2, Piezo Buzzer (PD14, PD15, PB15 respectivley)
void configure_MISC_GPIO(void) {
  GPIOD->MODER   &= (~(GPIO_MODER_MODE14_Msk | GPIO_MODER_MODE15_Msk));
  GPIOD->MODER   |= ((0x1 << GPIO_MODER_MODE14_Pos) | (0x1 << GPIO_MODER_MODE15_Pos));
  GPIOD->OTYPER  &= (uint16_t)(~(GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15));                        // sets 0xboth as push-pull
  GPIOD->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED14_Msk | GPIO_OSPEEDR_OSPEED15_Msk));                // clears Port 14 and 15 section
  GPIOD->OSPEEDR |= ((0x2 << GPIO_OSPEEDR_OSPEED14_Pos) | (0x2 << GPIO_OSPEEDR_OSPEED15_Pos)); // sets slew rate as high speed

  //	GPIOD->PUPDR &= (~(GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15)); // clears register for port 14 and 15
  //	GPIOD->PUPDR |= (0x00<<GPIO_PUPDR_PUPD14_Pos | 0x00<<GPIO_PUPDR_PUPD15_Pos ); // shifts 00 into and pos for 14 and 15
  GPIOD->ODR &= (~(GPIO_ODR_OD14 | GPIO_ODR_OD15));     // turns LEDs off

  GPIOB->MODER   &= (~(GPIO_MODER_MODE15_Pos));         // clears pos 15 of port B moder R reg
  GPIOB->MODER   |= (0x1 << GPIO_MODER_MODE15_Pos);     // sets pos 15 to general purpose output
  GPIOB->OTYPER  &= (~(GPIO_OTYPER_OT15_Msk));          // sets port B 15 to push-pull
  GPIOB->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED15_Msk));     // clears pos 15 in Ospeed R reg
  GPIOB->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED15_Pos); // sets slew rate to highspeed
  GPIOB->ODR     &= (~(GPIO_ODR_OD15));                 // turns peizer buzzer IO to low

  // configure PB1 as input for B switch
  GPIOB->MODER   &= (~(GPIO_MODER_MODE1_Pos));
  GPIOB->OTYPER  &= (~(GPIO_OTYPER_OT1_Msk));          // sets port B 15 to push-pull
  GPIOB->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED1_Msk));     // clears pos 15 in Ospeed R reg
  GPIOB->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED1_Pos); // sets slew rate to highspeed
  GPIOB->PUPDR   &= (~(GPIO_PUPDR_PUPD1));             // clears PUPDR for PB1
  GPIOB->PUPDR   |= (0x01 << GPIO_PUPDR_PUPD1_Pos);    // set pull up resistor for PB1
}

// ===============================================================
//                           CAN BUS
// ===============================================================

uint8_t CAN_RX(struct CAN_RX_data *CAN) {
  if (CAN->CAN_number == 1) {
    if (CAN1->RF1R & 0x3) {                                           // checks to see if there was a CAN message recieved
      CAN->address  = (CAN1->sFIFOMailBox[1].RIR & 0xFFE00040) >> 21; // extracts the address from the CAN message and stores it into the CAN
      CAN->dataL    = CAN1->sFIFOMailBox[1].RDLR;                     // extracts the LSB 4 bytes
      CAN->dataH    = CAN1->sFIFOMailBox[1].RDHR;                     // extracts the MSB 4 bytes
      CAN1->RF0R   |= 1 << 5;
      // clear bits 3 and 4 (indicating the mailboxes are full)
      CAN1->RF0R &= (unsigned int)~(1 << 3);
      CAN1->RF0R &= (unsigned int)~(1 << 4);
      return 1;
    } else
      return 0;                      // returns 0 if nothing recieved
  } else if (CAN->CAN_number == 2) { // samething but for CAN2
    if (CAN2->RF1R & 0x3) {
      CAN->address  = (CAN2->sFIFOMailBox[1].RIR & 0xFFE00040) >> 21;
      CAN->dataL    = CAN2->sFIFOMailBox[1].RDLR;
      CAN->dataH    = CAN2->sFIFOMailBox[1].RDHR;
      CAN2->RF0R   |= 1 << 5;
      // clear bits 3 and 4 (indicating the mailboxes are full)
      CAN2->RF0R &= (unsigned int)~(1 << 3);
      CAN2->RF0R &= (unsigned int)~(1 << 4);
      return 1;
    } else
      return 0; // returns 0 if nothing recieved
  } else
    return 255;
}

uint8_t CAN_TX(uint8_t CAN, uint8_t data_length, unsigned int dataH, unsigned int dataL, unsigned int address) {
  uint8_t mailbox = find_empty_CAN_TX_mailbox(CAN);
  if (mailbox == 255)
    return 250;
  if (CAN == 1) {
    CAN1->sTxMailBox[mailbox].TDHR = 0;                         // stores the dataH into the Mailbox to transmit
    CAN1->sTxMailBox[mailbox].TDLR = 0;                         // stores the dataL into the Mailbox to transmit
    CAN1->sTxMailBox[mailbox].TDTR = 0;                         // puts in the data length
    CAN1->sTxMailBox[mailbox].TDHR = (unsigned int)dataH;       // stores the dataH into the Mailbox to transmit
    CAN1->sTxMailBox[mailbox].TDLR = (unsigned int)dataL;       // stores the dataL into the Mailbox to transmit
    CAN1->sTxMailBox[mailbox].TDTR = (unsigned int)data_length; // puts in the data length
    // unsigned int CAN_TIR = CAN1->sTxMailBox[mailbox].TIR;
    // CAN_TIR &=	(unsigned int)~(0xFFE00000);
    // CAN_TIR |= (uint8_t) (address << 21);// enters in the CAN identifer
    CAN1->sTxMailBox[mailbox].TIR = 0;
    CAN1->sTxMailBox[mailbox].TIR = (address << 21); // enters in the CAN identifer
                                                     // CAN1->sTxMailBox[mailbox].TIR |= CAN_TIR;
    CAN1->sTxMailBox[mailbox].TIR |= (1 << 0);       // requested transmission
    while (1)                                        /// add timer in here for timeout
    {
      if ((CAN1->TSR & (1 << (1))))
        return 0;                                    // successful
      else if ((CAN1->TSR & (1 << 3)))
        CAN1->TSR |= (unsigned int)((1 << 7));
      CAN1->TSR |= (unsigned int)((1 << 7));         // used to abort the transmissions if there is an error
      return 1;                                      // TX error
    }
    return 255;                                      // timeout error
  }
  if (CAN == 2) {
    CAN2->sTxMailBox[mailbox].TDHR  = dataH;
    CAN2->sTxMailBox[mailbox].TDLR  = dataL;
    CAN2->sTxMailBox[mailbox].TDTR  = data_length;
    CAN2->sTxMailBox[mailbox].TIR   = address << 21;
    CAN2->sTxMailBox[mailbox].TIR  |= (1 << 0); // requested transmission
    while (1)                                   /// add timer in here for timeout
    {
      if ((CAN2->TSR & (1 << (1))))
        return 0;                               // successful
      else if ((CAN2->TSR & (1 << 3)))
        CAN2->TSR |= (unsigned int)((1 << 7));
      CAN2->TSR |= (unsigned int)((1 << 7));
      return 1;                                 // TX error
    }
    return 255;                                 // timeout error
  } else
    return 100;
}
uint8_t find_empty_CAN_TX_mailbox(uint8_t CAN) {
  return 0; // it is simply going to reload to the single mailbox and send
  volatile uint32_t *CAN_address = &CAN1->TSR;
  if (!((CAN == 1) || (CAN == 2)))
    return 100;
  if (CAN == 2) {
    CAN_address = &CAN2->TSR;
  }

  for (uint8_t i = 0; i < 3; i++) {
    if (*CAN_address & (1 << (i + 26))) {
      return i; // returns the mailbox number
    }
  }
  return 255;   // if mailboxes are all full
}

void CANGPIO_config() {
  // config AF functions
  // CAN1 RX PA11 TXPA12 CAN2 RX PB12 TX PB13 both AF9
  // Configure to Alternate Functions
  GPIOA->MODER &= (uint32_t)~0x3C00000;
  GPIOB->MODER &= (uint32_t)~0xF000000;
  GPIOA->MODER |= 0x2800000;
  GPIOB->MODER |= 0xA000000;
  // Configure the TX to be push-pull
  GPIOA->OTYPER &= (uint32_t)~0x1000;
  GPIOB->OTYPER &= (uint32_t)~0x2000;
  // Output speed
  GPIOA->OSPEEDR |= 0x3000000;
  GPIOB->OSPEEDR |= 0xC000000;
  // set the pull-ups
  GPIOA->PUPDR &= 0xC00000;
  GPIOB->PUPDR &= 0x3000000;
  // GPIOB->PUPDR |= 0x1000000;
  // GPIOA->PUPDR |= 0x400000;
  //  Remap the AFIO
  GPIOA->AFR[1] &= (uint32_t) ~(0xFF000);
  GPIOA->AFR[1] |= (0x99000);
  GPIOB->AFR[1] &= (uint32_t) ~(0xFF0000);
  GPIOB->AFR[1] |= (0x990000);
}

void CAN_Peripheral_config() {
  // volatile uint32_t* CAN = CAN1->MCR;
  // for (uint8_t x = 0; x > 2; x++){
  CAN1->MCR |= 0x8000;                     // reset CAN
  while (((CAN1->MCR & (CAN_MCR_RESET)))); // wait until reset

  CAN1->MCR |= 0x1;
  while (!(CAN1->MSR & 1));                // change
  CAN1->BTR &= (uint32_t) ~(0xC37F03FF);   // clears all bit timing bits and disables loop back and silent mode
  CAN1->BTR |= 0x22B0014;                  // enters the Bitrate as 125kb/s
  CAN1->MCR &= ~(CAN_MCR_SLEEP);           // Clear sleep bit
  CAN1->MCR &= (uint32_t) ~(1 << 0);       // places CAN into normal mode
  while ((CAN1->MSR & (1 << 0)));          // change for MSR

  CAN1->FMR   |= 0x1;                      // sets the filter initialisation to 'on'
  CAN1->FM1R  &= (uint32_t) ~(0x1);        // sets to mask mode filter
  CAN1->FS1R  |= 0x1;                      // sets to 32 bit mask, as the FR1/2 register is then for a single mask
  CAN1->FFA1R |= 0x1;                      // Assigned to FIFO 1
  CAN1->FS1R  |= (1 << 25);                // sets to 32 bit mask, as the FR1/2 register is then for a single mask
  CAN1->FFA1R |= (1 << 25);                // Assigned to FIFO 1
  CAN1->FA1R  &= 0xF0000000;               // disable all filters
  //
  CAN1->sFilterRegister[0].FR1  = 0; // assign filters so it will filter nothing
  CAN1->sFilterRegister[0].FR2  = 0;
  CAN1->sFilterRegister[25].FR1 = 0; // assign filters so it will filter nothing
  CAN1->sFilterRegister[25].FR2 = 0;

  CAN1->FA1R |= 0x1;                 // enable the filter
  CAN1->FA1R |= 1 << 25;             // enable the filter
  CAN1->FMR  &= (uint32_t) ~(0x1);   // take out of initialisation mode

  // this makes that all CAN messages will go to FIFO1
  CAN2->MCR |= 0x8000;                     // reset CAN
  while (((CAN1->MCR & (CAN_MCR_RESET)))); // wait until reset

  CAN2->MCR |= 0x1;
  while (!(CAN2->MSR & 1));                // change
  CAN2->BTR &= (uint32_t) ~(0xC37F03FF);   // clears all bit timing bits and disables loop back and silent mode
  CAN2->BTR |= 0x22B0014;                  // enters the Bitrate
  CAN2->MCR &= ~(CAN_MCR_SLEEP);           // Clear sleep bit
  CAN2->MCR &= (uint32_t) ~(1 << 0);       // places CAN into normal mode
  while ((CAN2->MSR & (1 << 0)));          // change for MSR
  CAN2->FMR &= (uint32_t) ~(0x1);          // take out of initialisation mode

  // CAN2->FMR |= 0x1; // sets the filter initialisation to 'on'
  // CAN2->FM1R &= (uint32_t)~(0x1); // sets to mask mode filter
  // CAN2->FS1R |= (1 << 25); // sets to 32 bit mask, as the FR1/2 register is then for a single mask
  // CAN2->FFA1R |= (1 << 25); // Assigned to FIFO 1
  // CAN2->FA1R &= 0xF0000000; // disable all filters
  // CAN2->sFilterRegister[0].FR1 = 0;// assign filters so it will filter nothing
  // CAN2->sFilterRegister[0].FR2 = 0;
  // CAN2->FA1R |= 0x1; // enable the filter

  // this makes that all CAN messages will go to FIFO1
}

// ===============================================================
//                            LORA
// ===============================================================

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

void configure_LoRa_module(void) {
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

  return;
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

// ===============================================================
//                           TIMERS
// ===============================================================

void TIM6init(void) {
  RCC->APB1ENR  |= RCC_APB1ENR_TIM6EN;
  RCC->APB1RSTR |= RCC_APB1RSTR_TIM6RST;
  __asm("NOP");
  __asm("NOP");
  RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM6RST);
  __asm("NOP");
  __asm("NOP");
  TIM6->CR1 |= TIM_CR1_OPM;
  TIM6->PSC |= 20;
  TIM6->ARR &= (~(TIM_ARR_ARR_Msk));
  TIM6->ARR |= 0X20; // 0.0005S delay
}

void TIM7init(void) {
  RCC->APB1ENR  |= RCC_APB1ENR_TIM7EN;
  RCC->APB1RSTR |= RCC_APB1RSTR_TIM7RST;
  __asm("NOP");
  __asm("NOP");
  RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM7RST);
  __asm("NOP");
  __asm("NOP");
  TIM7->CR1 |= TIM_CR1_OPM;
  TIM7->PSC |= 1399;
  TIM7->ARR &= (~(TIM_ARR_ARR_Msk));
  TIM7->ARR |= 60000; // 1s delay
}

                      // ===============================================================
//                          FLASH
// ===============================================================

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

// ===============================================================
//                           MISC
// ===============================================================

void buzzer(void) {
  TIM6->ARR &= (~(TIM_ARR_ARR_Msk));
  TIM6->PSC &= (~(TIM_PSC_PSC_Msk));
  TIM6->ARR |= 80;
  TIM6->PSC |= 128;
  TIM7->CR1 |= TIM_CR1_CEN;
  TIM6->CR1 |= TIM_CR1_CEN;     // ensures timer is enabled
  while ((TIM7->SR & TIM_SR_UIF) == 0) {
    GPIOB->ODR ^= 0X8000;
    while ((TIM6->SR & TIM_SR_UIF) == 0);
    TIM6->SR  &= ~(TIM_SR_UIF); // clears UIF
    TIM6->ARR |= 80;
    TIM6->CR1 |= TIM_CR1_CEN;   // Enables counter
  }
  TIM7->SR &= ~(TIM_SR_UIF);
}
