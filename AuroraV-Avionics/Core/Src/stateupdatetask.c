#include "tasks.h" 
#include "stm32f4xx.h"
#include "gpioControl.h" 
#include "FreeRTOS.h"
#include "task.h"


// State Update and Event Logging Task Function
void stateupdatetask(void const *argument) {
	
		TickType_t xLastWakeTime;
		const TickType_t xFrequency = pdMS_TO_TICKS(250);
		 
    for(;;) {
		
				// to execute in the same 500hz interval, but lower priority than high and low res tasks
					
				vTaskDelayUntil(&xLastWakeTime, xFrequency);	
			
    }
}
