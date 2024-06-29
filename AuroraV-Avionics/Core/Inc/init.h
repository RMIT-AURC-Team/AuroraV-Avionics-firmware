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

// may remove the #includes above
// peripherals
// CAN
// APB1 : CAN1, CAN2, UART3, SPI2, SPI3, TIM6, TIM7,
// clocks enabled and perihperals are reset in the following function
void configure_RCC_APB1();

// APB2 : SPI1, USART6
void configure_RCC_APB2();

// AHB1 : GPIO A, B, C, D, E
void configure_RCC_AHB1();

// General GPIO Configure for MISC: LED1, LED2, Piezo Buzzer (PD14, PD15, PB15
// respectivley)
void configure_MISC_GPIO();

// configure UART 3 - used for the GPS
void configure_UART3_GPS();

// configure USART 6 for serial to mini USB
void configure_UART6_Serial_2_mini_USB();

// configure SPI 1 for Accelerometers, Gyros, Barometers, Magnometer
void configure_SPI1_Sensor_Suite();

// configure SPI 3 for LoRa communications
void configure_SPI3_LoRa();

// configure SPI 2 for Flash
void configure_SPI2_Flash();

void configure_external_interrupts();

// configure ports for JTAG, might already be done
void EXTI1_IRQHandler();

#endif
