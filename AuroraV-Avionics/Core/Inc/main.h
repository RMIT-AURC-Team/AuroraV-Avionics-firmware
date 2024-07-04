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

#include "drivers.h"

#include "kalmanfilter.h"
#include "membuff.h"
#include "quaternion.h"
#include "state.h"

void vFlashBuffer(void *pvParameters);
void vDataAcquisitionH(void *pvParameters);
void vDataAcquisitionL(void *pvParameters);
void vStateUpdate(void *pvParameters);

void Error_Handler(void);

#define GYRO_SENSITIVITY           (0.00875f)
#define GYRO_CTRL_REG1             0x20
#define GYRO_CTRL_REG1_ODR_800Hz   0xC0
#define GYRO_CTRL_REG1_PD_ENABLE   0x08
#define GYRO_CTRL_REG1_AXIS_ENABLE 0x07

#define MAGNET_SENSITIVITY 			(1.0f / 1711.0f)
#define MAGNET_CTRL_REG1 	 			0x20
#define MAGNET_CTRL_REG1_FAST 	0x02
#define MAGNET_CTRL_REG2 				0x21
#define MAGNET_CTRL_REG2_FS16   0x60

#define ACCEL_SENSITIVITY_32G (1.0f / 1024.0f)
#define ACCEL_CNTL1           0x1B
#define ACCEL_ODCNTL          0x21
#define ACCEL_ODCNTL_RESERVED 0x90

#define BARO_TEMP_SENSITIVITY  (1.0f / 65535)
#define BARO_PRESS_SENSITIVITY (1.0f / 64)
#define BARO_ODR_CFG 					 0x37
#define BARO_ODR_CFG_PWR 			 0x01
#define BARO_ODR_CFG_DEEP_DIS	 0x80
#define BARO_OSR_CFG_RESERVED  0x80
#define BARO_OSR_CFG 					 0x36
#define BARO_OSR_CFG_PRESS_EN  0x40

// Dataframe header definitions
#define HEADER_ID_Pos     0x06
#define HEADER_LENGTH_Pos 0x00

#define HEADER_HIGHRES_ID     0x01
#define HEADER_HIGHRES_LENGTH 0x14
#define HEADER_HIGHRES        (HEADER_HIGHRES_ID << HEADER_ID_Pos) | HEADER_HIGHRES_LENGTH
#define HEADER_LOWRES_ID      0x02
#define HEADER_LOWRES_LENGTH  0x0A
#define HEADER_LOWRES         (HEADER_LOWRES_ID << HEADER_ID_Pos) | HEADER_LOWRES_LENGTH

#define LD2_PORT GPIOB
#define LD2_PIN  Pin7
#define LD3_PORT GPIOB
#define LD3_PIN  Pin14

#endif
