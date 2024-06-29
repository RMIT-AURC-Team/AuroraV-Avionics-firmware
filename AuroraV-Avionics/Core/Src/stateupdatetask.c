#include "tasks.h" 
#include "stm32f4xx.h"
#include "gpioControl.h" 
#include "FreeRTOS.h"
#include "task.h"


// State Update and Event Logging Task Function
void stateupdatetask(void const *argument) {
    for(;;) {
		
				// to execute in the same 500hz interval, but lower priority than high and low res tasks
					
				// 3. Wait for Timer Interrupt
				//osSignalWait(0x01, osWaitForever); 
					
			
    }
}
