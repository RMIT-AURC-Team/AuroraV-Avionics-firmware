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
