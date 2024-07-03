/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "gpioControl.h"
#include "init.h"
#include "main.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include "task.h"

#include "kalmanfilter.h"
#include "membuff.h"
#include "quaternion.h"

#include "accelX.h"
#include "gyroX.h"
#include "gyroY.h"
#include "gyroZ.h"

void GPIO_Init(void);
void vDataAcquisitionH(void *pvParameters);
void vUARTDebug(void *pvParameters);

void Error_Handler(void);

// Dataframe header definitions
#define HEADER_ID_Pos     0x06
#define HEADER_LENGTH_Pos 0x00

#define HEADER_HIGHRES_ID     0x01
#define HEADER_HIGHRES_LENGTH 0x01
#define HEADER_HIGHRES        (HEADER_HIGHRES_ID << HEADER_ID_Pos) | HEADER_HIGHRES_LENGTH
#define HEADER_LOWRES_ID      0x02
#define HEADER_LOWRES_LENGTH  0x0A
#define HEADER_LOWRES         (HEADER_LOWRES_ID << HEADER_ID_Pos) | HEADER_LOWRES_LENGTH

#define LD2_PORT GPIOB
#define LD2_PIN  Pin7
#define LD3_PORT GPIOB
#define LD3_PIN  Pin14

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
