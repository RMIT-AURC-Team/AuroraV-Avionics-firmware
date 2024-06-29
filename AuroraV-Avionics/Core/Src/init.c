#include "init.h"

// configure RCC APB1 : CAN1, CAN2, UART3, SPI2, SPI3, TIM6, TIM7,
void configure_RCC_APB1() {
  RCC->APB1ENR |= RCCAPB1(EN);
  RCC->APB1RSTR |= RCCAPB1(RST);
  __ASM("NOP");
  __ASM("NOP");
  RCC->APB1RSTR &= (uint16_t)(~(RCCAPB1(RST)));
  __ASM("NOP");
  __ASM("NOP");
}

// configure RCC APB2 : SPI1, USART6
void configure_RCC_APB2() {
  RCC->APB2ENR |= RCCAPB2(EN);
  RCC->APB2RSTR |= RCCAPB2(RST);
  __ASM("NOP");
  __ASM("NOP");
  RCC->APB2RSTR &= (uint16_t)(~(RCCAPB1(RST)));
  __ASM("NOP");
  __ASM("NOP");
}

// configure RCC AHB1 GPIO A, B, C, D, E
void configure_RCC_AHB1() {
  RCC->AHB1ENR |= RCCAHB1(EN);
  RCC->AHB1RSTR |= RCCAHB1(RST);
  __ASM("NOP");
  __ASM("NOP");
  RCC->AHB1RSTR &= (uint16_t)(~(RCCAHB1(RST)));
}

// General GPIO Configure for MISC: LED1, LED2, Piezo Buzzer (PD14, PD15,
// PB15 respectivley)
void configure_MISC_GPIO() {
  GPIOD->MODER &= (~(GPIO_MODER_MODE14_Msk | GPIO_MODER_MODE15_Msk));
  GPIOD->MODER |= ((0x1 << GPIO_MODER_MODE14_Pos) | (0x1 << GPIO_MODER_MODE15_Pos));
  GPIOD->OTYPER &= (uint16_t)(~(GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15));                         // sets 0xboth as push-pull
  GPIOD->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED14_Msk | GPIO_OSPEEDR_OSPEED15_Msk));                // clears Port 14 and 15 section
  GPIOD->OSPEEDR |= ((0x2 << GPIO_OSPEEDR_OSPEED14_Pos) | (0x2 << GPIO_OSPEEDR_OSPEED15_Pos)); // sets slew rate as high speed

  // GPIOD->PUPDR &= (~(GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15)); // clears
  // register for port 14 and 15 	GPIOD->PUPDR |=
  // (0x00<<GPIO_PUPDR_PUPD14_Pos | 0x00<<GPIO_PUPDR_PUPD15_Pos ); // shifts 00
  // into and pos for 14 and 15
  GPIOD->ODR &= (~(GPIO_ODR_OD14 | GPIO_ODR_OD15));     // turns LEDs off

  GPIOB->MODER &= (~(GPIO_MODER_MODE15_Pos));           // clears pos 15 of port B moder R reg
  GPIOB->MODER |= (0x1 << GPIO_MODER_MODE15_Pos);       // sets pos 15 to general purpose output
  GPIOB->OTYPER &= (~(GPIO_OTYPER_OT15_Msk));           // sets port B 15 to push-pull
  GPIOB->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED15_Msk));     // clears pos 15 in Ospeed R reg
  GPIOB->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED15_Pos); // sets slew rate to highspeed
  GPIOB->ODR &= (~(GPIO_ODR_OD15));                     // turns peizer buzzer IO to low

  // configure PB1 as input for B switch
  GPIOB->MODER &= (~(GPIO_MODER_MODE1_Pos));
  GPIOB->OTYPER &= (~(GPIO_OTYPER_OT1_Msk));           // sets port B 15 to push-pull
  GPIOB->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED1_Msk));     // clears pos 15 in Ospeed R reg
  GPIOB->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED1_Pos); // sets slew rate to highspeed
  GPIOB->PUPDR &= (~(GPIO_PUPDR_PUPD1));               // clears PUPDR for PB1
  GPIOB->PUPDR |= (0x01 << GPIO_PUPDR_PUPD1_Pos);      // set pull up resistor for PB1
}

void configure_external_interrupts() {
  __disable_irq();
  // PB0 as interrupt
  EXTI->IMR |= EXTI_IMR_IM1;   // enable port B bit 1 as interupt
  EXTI->FTSR |= EXTI_FTSR_TR1; // turn on falling edge interrupt
  NVIC_EnableIRQ(EXTI1_IRQn);
  NVIC_SetPriority(EXTI1_IRQn, 2);
  __enable_irq();
}
void EXTI1_IRQHandler() {}
