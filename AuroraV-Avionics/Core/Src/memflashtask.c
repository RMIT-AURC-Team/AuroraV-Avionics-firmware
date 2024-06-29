#include "tasks.h" 
#include "stm32f4xx.h"
#include "gpioControl.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


// Memory Flash Task Function
void memflashtask(void const *argument) {
		
//		// Create Mutex Handle for saving to flash
//		SemaphoreHandle_t xFlashMutex; 
		
		
//    for(;;) {
//			
//			  // Take mutex
//        xSemaphoreTake(xFlashMutex, portMAX_DELAY); 

//        // Write buffer to flash
//        // ... (your flash writing code) ...
//			
//        // Release mutex
//        xSemaphoreGive(xFlashMutex); 

//			
//    }
}
