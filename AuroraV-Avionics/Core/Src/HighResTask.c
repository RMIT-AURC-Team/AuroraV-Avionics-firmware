#include "tasks.h" 
#include "stm32f4xx.h"
#include "gpioControl.h" 
#include "FreeRTOS.h"
#include "task.h" 
#include "accelX.h"
#include "gyroX.h"
#include "gyroY.h"
#include "gyroZ.h"
#include "buffer.h"
#include "membuff.h"

// High-Resolution Task Function
void highrestask(void const *argument) {

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(250);
	unsigned int index = 0;

	for(;;) {
		GPIOB->ODR ^= 0x01 << 7; 	// Toggle PB7
		
		// Read in dummy sensor data. (processed data)
		
		/****** turn off optimiser for this step through in debug ******/
		float accelData = accelX[index];
		float gyroXData = gyroX[index];
		float gyroYData = gyroY[index];
		float gyroZData = gyroZ[index];
		
		// Store in Buff 
		
		// Convert float data to bytes and store each byte in the buffer - quick test for storing into buff. 
		uint8_t *accelDataBytes = (uint8_t*)&accelData;
		uint8_t *gyroXDataBytes = (uint8_t*)&gyroXData;
		uint8_t *gyroYDataBytes = (uint8_t*)&gyroYData;
		uint8_t *gyroZDataBytes = (uint8_t*)&gyroZData;

		for (int i = 0; i < sizeof(float); i++) {
				membuff.append(&membuff, accelDataBytes[i]);
				membuff.append(&membuff, gyroXDataBytes[i]);
				membuff.append(&membuff, gyroYDataBytes[i]);
				membuff.append(&membuff, gyroZDataBytes[i]);
		}
		
		// Calc quaternions 
		
		
		
		index++; 
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency); 		
	}
}
