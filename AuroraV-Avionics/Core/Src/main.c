
/********************************************
*			STM32F439 Main (C Startup File)  			*
*			Developed for the STM32								*
*			Author: 															*
*			Source File														*
********************************************/

#include <stdint.h>
#include "boardSupport.h"
#include "main.h"
#include "stm32f4xx.h"
#include "gpioControl.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "kalmanfilter.h"
#include "init.h"

//--------------------------------

// LD2 is on PB7
#define LD2_PORT GPIOB
#define LD2_PIN Pin7

// LD3 is on PB14
#define LD3_PORT GPIOB
#define LD3_PIN Pin14

// Task Handles 
TaskHandle_t highrestaskHandle = NULL;  
TaskHandle_t lowrestaskHandle = NULL; 
TaskHandle_t stateupdatetaskHandle = NULL; 
TaskHandle_t memflashtaskHandle = NULL; 

// Function Prototypes
void GPIO_Init(void);
void TIM1_Init(void);
void TIM2_Init(void);
void highrestask(void *pvParameters);
void lowrestask(void *pvParameters);
void stateupdatetask(void *pvParameters);
void memflashtask(void *pvParameters);

// Unsure of actual fix for linker error
// temporary (lol) solution
void _init(){}

int main(void)
{
	// Inits ********************************************
	configure_RCC_APB1();
	configure_RCC_APB2();
	configure_RCC_AHB1();
	
	SystemInit(); 
	//***************************************************	

	// Create Tasks *************************************

	// Create High-Resolution Task
	xTaskCreate(highrestask, "highrestask", 128, NULL, configMAX_PRIORITIES - 1, &highrestaskHandle);

	// Create Low-Resolution Task
	xTaskCreate(lowrestask, "lowrestask", 128, NULL, configMAX_PRIORITIES - 1, &lowrestaskHandle);
	
	// Create State Update and Event log Task 
	//xTaskCreate(stateupdatetask, "stateupdatetask", 128, NULL, configMAX_PRIORITIES - 3, &stateupdatetaskHandle);

	// Create Memory Flash Task
	//xTaskCreate(memflashtask, "memflashtask", 128, NULL, configMAX_PRIORITIES - 4, &memflashtaskHandle);
	
	//***************************************************
	
    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    while(1){}
}

// Not sure if we need an Idle hook to execute anything? 
void vApplicationIdleHook(void)
{
	// use for flash? 
}





