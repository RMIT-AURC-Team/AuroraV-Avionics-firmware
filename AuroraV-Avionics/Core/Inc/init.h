#ifndef _INIT_H
#define _INIT_H

#include "stm32f439xx.h"

#define RCCAPB1(reg) (             \
    RCC_APB1##reg##R_CAN1##reg     \
    | RCC_APB1##reg##R_CAN2##reg   \
    | RCC_APB1##reg##R_USART3##reg \
    | RCC_APB1##reg##R_SPI3##reg   \
    | RCC_APB1##reg##R_SPI2##reg   \
    | RCC_APB1##reg##R_TIM6##reg   \
    | RCC_APB1##reg##R_TIM7##reg   \
)

#define RCCAPB2(reg) (             \
    RCC_APB2##reg##R_SPI1##reg     \
    | RCC_APB2##reg##R_USART6##reg \
)

#define RCCAHB1(reg) (            \
    RCC_AHB1##reg##R_GPIOA##reg   \
    | RCC_AHB1##reg##R_GPIOB##reg \
    | RCC_AHB1##reg##R_GPIOC##reg \
    | RCC_AHB1##reg##R_GPIOD##reg \
    | RCC_AHB1##reg##R_GPIOE##reg \
)

void configure_RCC_APB1();
void configure_RCC_APB2();
void configure_RCC_AHB1();
void configure_MISC_GPIO();

void configure_external_interrupts();
void EXTI1_IRQHandler();

#endif
