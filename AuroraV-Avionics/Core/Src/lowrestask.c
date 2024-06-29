#include "tasks.h" 
#include "stm32f4xx.h"
#include "gpioControl.h" 
#include "FreeRTOS.h"
#include "task.h"

// Low-Resolution Task Function
void lowrestask(void const *argument) {

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(500);

	for(;;) {
		GPIOB->ODR ^= 0x01 << 14; 	// Toggle PB14
		vTaskDelayUntil(&xLastWakeTime, xFrequency); 	
	}
}
