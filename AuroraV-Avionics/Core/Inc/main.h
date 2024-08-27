/**
 * @author Matt Ricci
 * @file   main.h
 **/

#ifndef __MAIN_H
#define __MAIN_H

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "message_buffer.h"
#include "stdint.h"
#include "stdio.h"
#include "stm32f4xx.h"

#include "A3G4250D.h"
#include "BMP581.h"
#include "KX134_1211.h"
#include "gps.h"
#include "can.h"
#include "control.h"
#include "drivers.h"
#include "flash.h"
#include "lora.h"
#include "sensors.h"
#include "state.h"
#include "uart.h"

#include "kalmanfilter.h"
#include "membuff.h"
#include "quaternion.h"
#include "slidingwindow.h"

#include "accelX.h"
#include "accelY.h"
#include "accelZ.h"
#include "gyroX.h"
#include "gyroY.h"
#include "gyroZ.h"
#include "press.h"

void vFlashBuffer(void *pvParameters);
void vDataAcquisitionH(void *pvParameters);
void vDataAcquisitionL(void *pvParameters);
void vStateUpdate(void *pvParameters);
void vLoRaTransmit(void *pvParameters);
void vLoRaSample(void *pvParameters);
void vUsbReceive(void *pvParameters);
void vUsbTransmit(void *pvParameters);
void vGpsRead(void *pvParameters);

void configure_interrupts();
void Error_Handler(void);

/* ===================================================================== *
 *                      DATAFRAME HEADER DEFINITIONS                     *
 * ===================================================================== */

// GROUNDSTATION LORA
#define LORA_HEADER_AV_DATA 0x04
#define LORA_HEADER_GPS_DATA 0x05

// AEROBRAKES CAN
#define CAN_HEADER_AEROBRAKES_RETRACT 0x602
#define CAN_HEADER_AEROBRAKES_DATA 		0x601

// FLASH
#define HEADER_ID_Pos           0x06
#define HEADER_LENGTH_Pos       0x00
#define HEADER_EVENT_SUB_ID_Pos 0x04

#define HEADER_HIGHRES_ID       0x01
#define HEADER_HIGHRES_LENGTH   0x14
#define HEADER_HIGHRES          (HEADER_HIGHRES_ID << HEADER_ID_Pos) | HEADER_HIGHRES_LENGTH
#define HEADER_LOWRES_ID        0x02
#define HEADER_LOWRES_LENGTH    0x0A
#define HEADER_LOWRES           (HEADER_LOWRES_ID << HEADER_ID_Pos) | HEADER_LOWRES_LENGTH

#define HEADER_EVENT_ID         0x03
#define HEADER_EVENT_LENGTH     0x02
#define HEADER_EVENT_LAUNCH_ID  0x00
#define HEADER_EVENT_COAST_ID   0x01
#define HEADER_EVENT_APOGEE_ID  0x02
#define HEADER_EVENT_DESCENT_ID 0x03

#define HEADER_EVENT_LAUNCH     (HEADER_EVENT_ID << HEADER_ID_Pos | HEADER_EVENT_LAUNCH_ID << HEADER_EVENT_SUB_ID_Pos | HEADER_EVENT_LENGTH)
#define HEADER_EVENT_COAST      (HEADER_EVENT_ID << HEADER_ID_Pos | HEADER_EVENT_COAST_ID << HEADER_EVENT_SUB_ID_Pos | HEADER_EVENT_LENGTH)
#define HEADER_EVENT_APOGEE     (HEADER_EVENT_ID << HEADER_ID_Pos | HEADER_EVENT_APOGEE_ID << HEADER_EVENT_SUB_ID_Pos | HEADER_EVENT_LENGTH)
#define HEADER_EVENT_DESCENT    (HEADER_EVENT_ID << HEADER_ID_Pos | HEADER_EVENT_DESCENT_ID << HEADER_EVENT_SUB_ID_Pos | HEADER_EVENT_LENGTH)

/* ===================================================================== *
 *                           DEVICE DEFINITIONS                          *
 * ===================================================================== */

// ACCELEROMETER
#define ACCEL_PORT_1  GPIOA
#define ACCEL_CS_1    GPIO_ODR_OD1
// FLIGHT AXES
#define ACCEL_AXES_1  ((const uint8_t[]){0, 2, 1})
#define ACCEL_SIGN_1  ((const int8_t[]){1, 1, -1})
// DRONE AXES
//#define ACCEL_AXES_1  ((const uint8_t[]){0, 1, 2})
//#define ACCEL_SIGN_1  ((const int8_t[]){1, 1, -1})

#define ACCEL_PORT_2  GPIOB
#define ACCEL_CS_2    GPIO_ODR_OD0
#define ACCEL_AXES_2  ((const uint8_t[]){0, 2, 1})
#define ACCEL_SIGN_2  ((const int8_t[]){1, -1, 1})

// GYROSCOPE
#define GYRO_PORT     GPIOA
#define GYRO_CS       GPIO_ODR_OD2
#define GYRO_AXES     ((const uint8_t[]){0, 2, 1})
#define GYRO_SIGN     ((const int8_t[]){1, 1, 1})

// BAROMETER
#define BARO_PORT     GPIOA
#define BARO_CS       GPIO_ODR_OD3

// FLASH
#define FLASH_PORT    	 GPIOE
#define FLASH_CS      	 GPIO_ODR_OD11
#define FLASH_PAGE_SIZE  256
#define FLASH_PAGE_COUNT 65536

// LORA
#define LORA_PORT     GPIOD
#define LORA_CS       GPIO_ODR_OD0

// USB UART
#define USB_PORT      GPIOC
#define USB_INTERFACE USART6
#define USB_BAUD      921600

/* ===================================================================== *
 *                         EVENT GROUP DEFINITIONS                       *
 * ===================================================================== */

#define GROUP_MESSAGE_READY_LORA  0x01
#define GROUP_MESSAGE_READY_USB   0x02

#define GROUP_TASK_ENABLE_FLASH   0x01
#define GROUP_TASK_ENABLE_HIGHRES 0x02
#define GROUP_TASK_ENABLE_LOWRES  0x04
#define GROUP_TASK_ENABLE_LORA    0x08
#define GROUP_TASK_ENABLE_IDLE    0x80

/* ===================================================================== *
 *                            MISC DEFINITIONS                           *
 * ===================================================================== */

// Drone launch threshold
//#define ACCEL_LAUNCH         3.0f
// Flight launch threshold
#define ACCEL_LAUNCH         5.0f

#define MAIN_ALTITUDE_METERS 396.0f

#define SIGINT 					 	   0x03
#define BACKSPACE 					 0x08
#define LINE_FEED 					 0x0A
#define CARRIAGE_RETURN      0x0D

#endif
