/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           main.h
 * @brief          Header for main.c file.
 *                 This file contains the common defines of the application.
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

#include "FreeRTOS.h"
#include "event_groups.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include "task.h"

#include "A3G4250D.h"
#include "BMP581.h"
#include "KX134_1211.h"
#include "can.h"
#include "drivers.h"
#include "flash.h"
#include "lora.h"
#include "uart.h"
#include "sensors.h"
#include "state.h"

#include "kalmanfilter.h"
#include "membuff.h"
#include "quaternion.h"

void vFlashBuffer(void *pvParameters);
void vDataAcquisitionH(void *pvParameters);
void vDataAcquisitionL(void *pvParameters);
void vStateUpdate(void *pvParameters);
void vLoRaCommunicate(void *pvParameters);

void Error_Handler(void);

#define LORA_HEADER_AVD1 0x04
#define LORA_HEADER_AVD2 0x05
#define LORA_HEADER_AVD3 0x06

// Dataframe header definitions
#define HEADER_ID_Pos           0x06
#define HEADER_LENGTH_Pos       0x00
#define HEADER_EVENT_SUB_ID_Pos 0x04

#define HEADER_HIGHRES_ID     0x01
#define HEADER_HIGHRES_LENGTH 0x14
#define HEADER_HIGHRES        (HEADER_HIGHRES_ID << HEADER_ID_Pos) | HEADER_HIGHRES_LENGTH
#define HEADER_LOWRES_ID      0x02
#define HEADER_LOWRES_LENGTH  0x0A
#define HEADER_LOWRES         (HEADER_LOWRES_ID << HEADER_ID_Pos) | HEADER_LOWRES_LENGTH

#define HEADER_EVENT_ID         0x03
#define HEADER_EVENT_LENGTH     0x02
#define HEADER_EVENT_LAUNCH_ID  0x00
#define HEADER_EVENT_COAST_ID   0x01
#define HEADER_EVENT_APOGEE_ID  0x02
#define HEADER_EVENT_DESCENT_ID 0x03

#define HEADER_EVENT_LAUNCH  (HEADER_EVENT_ID << HEADER_ID_Pos | HEADER_EVENT_LAUNCH_ID << HEADER_EVENT_SUB_ID_Pos | HEADER_EVENT_LENGTH)
#define HEADER_EVENT_COAST   (HEADER_EVENT_ID << HEADER_ID_Pos | HEADER_EVENT_COAST_ID << HEADER_EVENT_SUB_ID_Pos | HEADER_EVENT_LENGTH)
#define HEADER_EVENT_APOGEE  (HEADER_EVENT_ID << HEADER_ID_Pos | HEADER_EVENT_APOGEE_ID << HEADER_EVENT_SUB_ID_Pos | HEADER_EVENT_LENGTH)
#define HEADER_EVENT_DESCENT (HEADER_EVENT_ID << HEADER_ID_Pos | HEADER_EVENT_DESCENT_ID << HEADER_EVENT_SUB_ID_Pos | HEADER_EVENT_LENGTH)

/* ===================================================================== *
 *                           DEVICE DEFINITIONS                          *
 * ===================================================================== */

#define ACCEL_LAUNCH 5

#define ACCEL_PORT_1 GPIOA
#define ACCEL_CS_1   GPIO_ODR_OD1
#define ACCEL_AXES_1 ((const uint8_t[]){0, 2, 1})
#define ACCEL_SIGN_1 ((const int8_t[]){1, 1, -1})

#define ACCEL_PORT_2 GPIOB
#define ACCEL_CS_2   GPIO_ODR_OD0
#define ACCEL_AXES_2 ((const uint8_t[]){0, 1, 2})
#define ACCEL_SIGN_2 ((const int8_t[]){1, 1, 1})

#define GYRO_PORT GPIOA
#define GYRO_CS   GPIO_ODR_OD2
#define GYRO_AXES ((const uint8_t[]){0, 2, 1})
#define GYRO_SIGN ((const int8_t[]){1, 1, 1})

#define BARO_PORT GPIOA
#define BARO_CS   GPIO_ODR_OD3

#define FLASH_PORT GPIOE
#define FLASH_CS   GPIO_ODR_OD11

#endif
