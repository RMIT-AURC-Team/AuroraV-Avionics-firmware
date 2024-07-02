#include "tasks.h" 
#include "stm32f4xx.h"
#include "gpioControl.h" 
#include "FreeRTOS.h"
#include "task.h" 
#include "accelX.h"
#include "gyroX.h"
#include "gyroY.h"
#include "gyroZ.h"

// High-Resolution Task Function
void highrestask(void const *argument) {

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(250);
	unsigned int index = 0;

	for(;;) {
		GPIOB->ODR ^= 0x01 << 7; 	// Toggle PB7
		
		float accelData = accelX[index];
		float gyroXData = gyroX[index];
		float gyroYData = gyroY[index];
		float gyroZData = gyroZ[index];
		
		
		index++; 
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency); 		
	}
}
