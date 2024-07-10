/*
 * gpio_struct.c
 *
 *  Created on: Jun 19, 2024
 *      Author: aidencontini
 */

#include "gpio_struct.h"

// Function to initialize the GPIO_Config struct
GPIO_Config create_GPIO_Config(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    GPIO_Config new_config;
    new_config.GPIOx = GPIOx;
    new_config.GPIO_Pin = GPIO_Pin;
    return new_config;
}
