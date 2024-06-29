#include "tasks.h" 
#include "stm32f4xx.h"
#include "gpioControl.h" 
#include "FreeRTOS.h"
#include "task.h" 

// High-Resolution Task Function
void highrestask(void const *argument) {

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(250);

	for(;;) {
		GPIOB->ODR ^= 0x01 << 7; 	// Toggle PB7
		vTaskDelayUntil(&xLastWakeTime, xFrequency); 	
	}
}
