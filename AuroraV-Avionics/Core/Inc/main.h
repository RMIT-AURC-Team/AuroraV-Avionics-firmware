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

#include "FreeRTOS.h"
#include "event_groups.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include "task.h"

#include "can.h"
#include "drivers.h"
#include "lora.h"
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
#define HEADER_EVENT_MOTOR_ID   0x01
#define HEADER_EVENT_APOGEE_ID  0x02
#define HEADER_EVENT_DESCENT_ID 0x03

#define HEADER_EVENT_LAUNCH  (HEADER_EVENT_ID << HEADER_ID_Pos | HEADER_EVENT_LAUNCH_ID << HEADER_EVENT_SUB_ID_Pos | HEADER_EVENT_LENGTH)
#define HEADER_EVENT_MOTOR   (HEADER_EVENT_ID << HEADER_ID_Pos | HEADER_EVENT_MOTOR_ID << HEADER_EVENT_SUB_ID_Pos | HEADER_EVENT_LENGTH)
#define HEADER_EVENT_APOGEE  (HEADER_EVENT_ID << HEADER_ID_Pos | HEADER_EVENT_APOGEE_ID << HEADER_EVENT_SUB_ID_Pos | HEADER_EVENT_LENGTH)
#define HEADER_EVENT_DESCENT (HEADER_EVENT_ID << HEADER_ID_Pos | HEADER_EVENT_DESCENT_ID << HEADER_EVENT_SUB_ID_Pos | HEADER_EVENT_LENGTH)

#define LD2_PORT GPIOB
#define LD2_PIN  Pin7
#define LD3_PORT GPIOB
#define LD3_PIN  Pin14

#endif
