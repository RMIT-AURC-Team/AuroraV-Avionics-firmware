	#include "tasks.h" 
	#include "stm32f4xx.h"
	#include "gpioControl.h" 
	#include "FreeRTOS.h"
	#include "task.h"
	#include "semphr.h"


	// Memory Flash Task Function
	void vApplicationIdleHook(void) {
		
			// Create Mutex Handle for saving to flash
			SemaphoreHandle_t xFlashMutex = xSemaphoreCreateMutex();
			
			// Check if mutex was created successfully
			if (xFlashMutex == NULL) {
					// Handle error: mutex creation failed
					// You can use an infinite loop or any error handling mechanism
					while (1);
			}
			
			
			 
			// Take mutex
			if (xSemaphoreTake(xFlashMutex, portMAX_DELAY) == pdTRUE) {
					// Write buffer to flash
					// ... (your flash writing code) ...

					// Release mutex
					xSemaphoreGive(xFlashMutex);
			}
					
					

	}
