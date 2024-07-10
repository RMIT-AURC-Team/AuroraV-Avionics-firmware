/*
 * gpio_struct.h
 *
 *  Created on: Jun 19, 2024
 *      Author: aidencontini
 */


#ifndef INC_GPIO_STRUCT_H_
#define INC_GPIO_STRUCT_H_
#include "stm32f4xx_hal.h"

typedef struct {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
} GPIO_Config;

GPIO_Config create_GPIO_Config(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

#endif /* INC_GPIO_STRUCT_H_ */
