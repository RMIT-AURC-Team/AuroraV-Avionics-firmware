
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
#include "membuff.h"
#include "Buffer.h"

#include "accelX.h"
#include "gyroX.h"
#include "gyroY.h"
#include "gyroZ.h"


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
TaskHandle_t iomontaskHandle = NULL;
TaskHandle_t memflashtaskHandle = NULL; 

// Function Prototypes
void GPIO_Init(void);
void TIM1_Init(void);
void TIM2_Init(void);
void highrestask(void *pvParameters);
void lowrestask(void *pvParameters);
void stateupdatetask(void *pvParameters);
void iomontask(void *pvParameters);
void memflashtask(void *pvParameters);

// Unsure of actual fix for linker error
// temporary (lol) solution
void _init(){}
	
// Init membuff 

	

int main(void)
{		
	
	// Inits ********************************************
	configure_RCC_APB1();
	configure_RCC_APB2();
	configure_RCC_AHB1();
	
	SystemInit(); 
	GPIO_Init();
	initBuff();
	//***************************************************	
	

	
	
	// Create Tasks *************************************

	// Create High-Resolution Task
	xTaskCreate(highrestask, "highrestask", 128, NULL, configMAX_PRIORITIES - 1, &highrestaskHandle);

	// Create Low-Resolution Task
	xTaskCreate(lowrestask, "lowrestask", 128, NULL, configMAX_PRIORITIES - 1, &lowrestaskHandle);
	
	// Create State Update and Event log Task 
	//xTaskCreate(stateupdatetask, "stateupdatetask", 128, NULL, configMAX_PRIORITIES - 3, &stateupdatetaskHandle);
	
	// Create I/O Monitor Task 
	//xTaskCreate(iomontask, "iomontask", 128, NULL, configMAX_PRIORITIES - 4, &iomontaskHandle);

	// Create Memory Flash Task
	//xTaskCreate(memflashtask, "memflashtask", 128, NULL, configMAX_PRIORITIES - 5, &memflashtaskHandle);
	
	//***************************************************
	
    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    while(1){}
}

// Not sure if we need an Idle hook to execute anything? 
void vApplicationIdleHook(void)
{
//    // For Example Turn off LD2 and LD3
//    gpio_resetGPIO(LD2_PORT, LD2_PIN);
//    gpio_resetGPIO(LD3_PORT, LD3_PIN);
}


void GPIO_Init(void)	{
	// Enable GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Initialize GPIO pins for LD2 (PB7) and LD3 (PB14)
    GPIO_Config gpioConfig;
    
    // LD2 (PB7)
    gpioConfig.port = LD2_PORT;
    gpioConfig.pin = LD2_PIN;
    gpioConfig.mode = GPIO_Output;
    gpioConfig.outputType = GPIO_Output_PushPull;
    gpioConfig.speed = GPIO_50MHz;
    gpioConfig.pullUpDown = GPIO_No_Pull;
    gpio_configureGPIO(&gpioConfig);
    
    // LD3 (PB14)
    gpioConfig.pin = LD3_PIN;
    gpio_configureGPIO(&gpioConfig);
	
		  // Flash LD2 (PB7) and LD3 (PB14) once
    gpio_setGPIO(LD2_PORT, LD2_PIN);    // Turn on LD2 (PB7)
    gpio_setGPIO(LD3_PORT, LD3_PIN);    // Turn on LD3 (PB14)
    gpio_resetGPIO(LD2_PORT, LD2_PIN);  // Turn off LD2 (PB7)
    gpio_resetGPIO(LD3_PORT, LD3_PIN);  // Turn off LD3 (PB14)
}



