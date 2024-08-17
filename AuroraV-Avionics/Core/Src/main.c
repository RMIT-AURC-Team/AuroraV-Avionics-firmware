/**
 * @author Matt Ricci
 * @file main.c
 * @todo Implement globals as context struct to pass to external functions.
 * e.g. passing context of flash, uart, etc. to control functions.
 **/

#include "main.h"

#ifdef DUMMY
	long hDummyIdx = 0;
	long lDummyIdx = 0;
#endif

// ============================
//           HANDLES
// ============================

// Task Handles
TaskHandle_t xDataAqcquisitionHHandle = NULL;
TaskHandle_t xDataAqcquisitionLHandle = NULL;
TaskHandle_t xFlashBufferHandle       = NULL;
TaskHandle_t xStateUpdateHandle       = NULL;
TaskHandle_t xLoRaTransmitHandle      = NULL;
TaskHandle_t xLoRaSampleHandle 				= NULL;
TaskHandle_t xUsbReceiveHandle        = NULL;

EventGroupHandle_t xTaskEnableGroup; // 0: FLASH,  1: HIGHRES, 2: LOWRES, 3: LORA, 7: IDLE
EventGroupHandle_t xMsgReadyGroup;   // 0: LORA, 1: USB

KX134_1211 lAccel_s;
KX134_1211 hAccel_s;
KX134_1211 *pAccel_s;
A3G4250D gyro_s;
BMP581 baro_s;

UART usb;
LoRa lora;
Flash flash;

// ============================
//          BUFFERS
// ============================

#define AVG_BUFF_SIZE 15
#define MSG_BUFF_SIZE 128
#define MEM_BUFF_SIZE 20992

// USB
const size_t xUsbBuffSize = MSG_BUFF_SIZE;
MessageBufferHandle_t xUsbTxBuff;
StreamBufferHandle_t xUsbRxBuff;
uint8_t usbRxBuff[MSG_BUFF_SIZE];
uint8_t usbRxBuffIdx = 0;

// LoRa
const size_t xLoRaBuffSize = MSG_BUFF_SIZE;
MessageBufferHandle_t xLoRaTxBuff;
MessageBufferHandle_t xLoRaRxBuff;

// Flash
MemBuff mem;
uint8_t buff[MEM_BUFF_SIZE];
uint8_t outBuff[FLASH_PAGE_SIZE];

// Averages
float avgPressCurrent  = 0;
float avgPressPrevious = 0;
SlidingWindow avgPress;
float avgPressBuff[AVG_BUFF_SIZE];

float avgVelCurrent  = 0;
float avgVelPrevious = 0;
SlidingWindow avgVel;
float avgVelBuff[AVG_BUFF_SIZE];

// ============================
//           DATA
// ============================

// Calculated attitude variables
Quaternion qRot;                // Global attitude quaternion
float vAttitude[3] = {0, 0, 1}; // Attitude vector
float zUnit[3]     = {0, 0, 1}; // Z unit vector
float cosine       = 0;         // Tilt angle cosine
float tilt         = 0;         // Tilt angle

// Flight dynamics state variables
float altitude          = 0;         // Current altitude
float velocity          = 0;         // Current vertical velocity

enum State currentState = PRELAUNCH; // Boot in prelaunch

int main(void) {
  // Bring up RCC
  configure_RCC_APB1();
  configure_RCC_APB2();
  configure_RCC_AHB1();
  configure_MISC_GPIO();
  configure_UART3_GPS();
  configure_SPI1_Sensor_Suite();
  configure_SPI3_LoRa();
  configure_SPI4_Flash();

  configure_interrupts();

  // Initialise timers
  TIM6init();
  TIM7init();

  // Configure peripherals
  CANGPIO_config();
  CAN_Peripheral_config();

	#ifdef TRACE
		xTraceEnable(TRC_START);
	#endif
	
	GPIOB->ODR |= 0X8000;

  Flash_init(&flash, FLASH_PORT, FLASH_CS, FLASH_PAGE_SIZE, FLASH_PAGE_COUNT);
  UART_init(&usb, USB_INTERFACE, USB_PORT, USB_BAUD, OVER8);
  xUsbRxBuff = xStreamBufferCreate(xUsbBuffSize, 1);

  // Initialise LoRa interface and message buffers
  LoRa_init(&lora, LORA_PORT, LORA_CS, BW500, SF9, CR5);
  xLoRaTxBuff = xMessageBufferCreate(xLoRaBuffSize);

  // Initialise sensors
  BMP581_init(&baro_s, BARO_PORT, BARO_CS, BMP581_TEMP_SENSITIVITY, BMP581_PRESS_SENSITIVITY);
  A3G4250D_init(&gyro_s, GYRO_PORT, GYRO_CS, A3G4250D_SENSITIVITY, GYRO_AXES, GYRO_SIGN);
  KX134_1211_init(&lAccel_s, ACCEL_PORT_1, ACCEL_CS_1, ACCEL_SCALE_LOW, ACCEL_AXES_1, ACCEL_SIGN_1);
  KX134_1211_init(&hAccel_s, ACCEL_PORT_2, ACCEL_CS_2, ACCEL_SCALE_HIGH, ACCEL_AXES_2, ACCEL_SIGN_2);
  pAccel_s = &lAccel_s;
	
	// Initialise sliding window average buffers
	SlidingWindow_init(&avgVel, avgVelBuff, AVG_BUFF_SIZE);
	SlidingWindow_init(&avgPress, avgPressBuff, AVG_BUFF_SIZE);

  // Send AB ground test message over CAN
  unsigned int CANHigh = 0;
  unsigned int CANLow  = 0;
  unsigned int id      = 0x603;
  CAN_TX(1, 8, CANHigh, CANLow, id);

  MemBuff_init(&mem, buff, MEM_BUFF_SIZE, FLASH_PAGE_SIZE);
  Quaternion_init(&qRot);

  xTaskEnableGroup = xEventGroupCreate();
  xMsgReadyGroup   = xEventGroupCreate();
	xEventGroupSetBits(xMsgReadyGroup, GROUP_MESSAGE_READY_LORA);

  // Create task handles
  xTaskCreate(vDataAcquisitionH, "HDataAcq", 256, NULL, configMAX_PRIORITIES - 2, &xDataAqcquisitionHHandle);
  xTaskCreate(vDataAcquisitionL, "LDataAcq", 256, NULL, configMAX_PRIORITIES - 3, &xDataAqcquisitionLHandle);
  xTaskCreate(vStateUpdate, "StateUpdate", 256, NULL, configMAX_PRIORITIES - 4, &xStateUpdateHandle);
  xTaskCreate(vFlashBuffer, "FlashData", 256, NULL, configMAX_PRIORITIES - 1, &xFlashBufferHandle);
  xTaskCreate(vLoRaTransmit, "LoRaTx", 256, NULL, configMAX_PRIORITIES - 5, &xLoRaTransmitHandle);
	xTaskCreate(vLoRaSample, "LoRaSample", 256, NULL, configMAX_PRIORITIES - 6, &xLoRaSampleHandle);
  xTaskCreate(vUsbReceive, "UsbRx", 256, NULL, configMAX_PRIORITIES - 6, &xUsbReceiveHandle);

  vTaskStartScheduler();
}

/* ===================================================================== *
 *                            STATE MANAGEMENT                           *
 * ===================================================================== */

void vStateUpdate(void *argument) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz

  unsigned int CANHigh        = 0;
  unsigned int CANLow         = 0;
  unsigned int id             = 0;

  for (;;) {
    // Block until 20ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Emergency aerobrakes for excessive tilt
    if (tilt >= 30.0f) {
      // CAN payload for aerobrakes retract
      CANHigh = 0x00000000;
      CANLow  = 0x00000000;
      id      = CAN_HEADER_AEROBRAKES_RETRACT;
      CAN_TX(1, 8, CANHigh, CANLow, id);
    }
		
    switch (currentState) {
    case PRELAUNCH:
      if (pAccel_s->accelData[ZINDEX] >= ACCEL_LAUNCH) {
				#ifdef FLIGHT_TEST 
					GPIOB->ODR ^= 0X8000; 
				#endif
				vTaskDelete(xUsbReceiveHandle);																	 // Ignore all USB communications
        xEventGroupSetBits(xTaskEnableGroup, GROUP_TASK_ENABLE_FLASH);   // Enable flash
				xEventGroupSetBits(xTaskEnableGroup, GROUP_TASK_ENABLE_HIGHRES); // Enable high resolution data acquisition
        xEventGroupSetBits(xTaskEnableGroup, GROUP_TASK_ENABLE_LOWRES);  // Enable low resolution data acquisition
        currentState = LAUNCH;
      }
      break;
			
    case LAUNCH:
			avgVel.calculateMovingAverage(&avgVel, &avgVelCurrent);
      // Send altitude to aerobrakes via CAN
      CANHigh = 0x00000000;
      CANLow  = (unsigned int)altitude;
      id      = CAN_HEADER_AEROBRAKES_DATA;
      CAN_TX(1, 8, CANHigh, CANLow, id);
      // Transition to motor burnout state on velocity decrease
      if ((avgVelCurrent - avgVelPrevious) < 0) {
				#ifdef FLIGHT_TEST 
					GPIOB->ODR ^= 0X8000; 
				#endif
        currentState = COAST;
      }
			avgVelPrevious = avgVelCurrent;
      break;
			
    case COAST:
			avgPress.calculateMovingAverage(&avgPress, &avgPressCurrent);
      // Send altitude to aerobrakes via CAN
      CANHigh = 0x00000000;
      CANLow  = (unsigned int) altitude;
      id      = CAN_HEADER_AEROBRAKES_DATA;
      CAN_TX(1, 8, CANHigh, CANLow, id);
      // Transition to apogee state on three way vote of altitude, velocity, and tilt
      // apogee is determined as two of three conditions evaluating true
      if ((((avgPressCurrent - avgPressPrevious) > 0) + (tilt >= 90) + (velocity < 0.0f)) >= 2) {
				#ifdef FLIGHT_TEST 
					GPIOB->ODR ^= 0X8000; 
				#endif
        currentState = APOGEE;
        // Send transmission to trigger apogee E-matches
      }
			avgPressPrevious = avgPressCurrent;
      break;
			
    case APOGEE:
			// Retract aerobrakes
		  CANHigh = 0x00000000;
      CANLow  = 0x00000000;
      id      = CAN_HEADER_AEROBRAKES_RETRACT;
      CAN_TX(1, 8, CANHigh, CANLow, id);
			// Transition to descent state when below main deployment altitude
      if (altitude <= MAIN_ALTITUDE_METERS) {
				#ifdef FLIGHT_TEST 
					GPIOB->ODR ^= 0X8000; 
				#endif
        currentState = DESCENT;
        // Add descent event dataframe to buffer
      }
      break;
			
    case DESCENT:
      // Handle descent state actions
      break;
    }		
  }
}

/* ===================================================================== *
 *                            FLASH HANDLING                             *
 * ===================================================================== */

// Use idle time to write flash
void vApplicationIdleHook(void) {
  // Write if a page is available in the buffer
  if (currentState >= LAUNCH && mem.pageReady)
    xEventGroupSetBits(xTaskEnableGroup, GROUP_TASK_ENABLE_FLASH);
}

void vFlashBuffer(void *argument) {
  const TickType_t timeout = pdMS_TO_TICKS(1);
  uint32_t pageAddr        = 0;
  for (;;) {
    // Wait for write flag to be ready, clear flag on exit
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, GROUP_TASK_ENABLE_FLASH, pdTRUE, pdFALSE, timeout);
    if (uxBits & GROUP_TASK_ENABLE_FLASH) {
      bool success = mem.readPage(&mem, outBuff); // Flush data to output buffer
      if (success) {
        // Write data to flash memory
        flash.writePage(&flash, pageAddr, outBuff);
        pageAddr += 0x100;
      }
    }
  }
}


/* ===================================================================== *
 *                             LORA HANDLING                             *
 * ===================================================================== */

/**
 * @brief LoRa transmit task
 *
 * @details Reads data from LoRa Tx buffer and sends to SX1272 for 
 * transmit.
 */
void vLoRaTransmit(void *argument) {
  const TickType_t blockTime = pdMS_TO_TICKS(250);
  uint8_t rxData[LORA_MSG_LENGTH];

  for (;;) {
		// Wait for SX1272 to be ready for transmission
		EventBits_t uxBits = xEventGroupWaitBits(xMsgReadyGroup, GROUP_MESSAGE_READY_LORA, pdFALSE, pdFALSE, blockTime);
		if ((uxBits & GROUP_MESSAGE_READY_LORA)) {
			
			// Wait to receive message in buffer
			size_t xReceivedBytes = xMessageBufferReceive(
				xLoRaTxBuff,
				(void *)rxData,
				sizeof(rxData),
				blockTime
			);		
				
			// Transmit if message is available
			if (xReceivedBytes) {
				lora.transmit(&lora, rxData);	
				xEventGroupClearBits(xMsgReadyGroup, GROUP_MESSAGE_READY_LORA);
			}				
		}	
  }
	
}

/**
 * @brief LoRa sample task
 *
 * @details Samples current sensor data from RAM every 250ms and
 * queues it to be handled by vLoRaTransmit.
 */
void vLoRaSample(void *argument) {
	TickType_t xLastWakeTime;
	const TickType_t blockTime = pdMS_TO_TICKS(0);
	const TickType_t xFrequency = pdMS_TO_TICKS(250);

	for(;;) {
		// Block until 250ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
	
		// Create AVData packet and send to buffer
		LoRa_Packet avData = LoRa_AVData(
			LORA_HEADER_AV_DATA, 
			currentState,
			lAccel_s.rawAccelData, 
			hAccel_s.rawAccelData, 
			KX134_1211_DATA_TOTAL, 
			gyro_s.rawGyroData,
			A3G4250D_DATA_TOTAL,
			altitude,
			velocity
		);
		
		xMessageBufferSend(xLoRaTxBuff, &avData, LORA_MSG_LENGTH, blockTime);
	}
}

/**
 * @brief LoRa Tx complete interrupt handler
 *
 * @details External interrupt on Tx complete signal from SX1272
 * tranceiver. 
 * @details On interrupt the LoRa ready flag is set in xMsgReadyGroup.
 */
void EXTI1_IRQHandler(void) {
  EXTI->PR |= (0x02);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult;

	xResult = xEventGroupSetBitsFromISR(
		xMsgReadyGroup, 
		GROUP_MESSAGE_READY_LORA, 
		&xHigherPriorityTaskWoken
	);

	if( xResult != pdFAIL )
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/* ===================================================================== *
 *                             UART HANDLING                             *
 * ===================================================================== */

/**
 * @brief USB receive task
 *
 * @details Reads data from UART Rx buffer and sends back to host 
 * for display.
 * @details On receiving carriage return, processes command stored 
 * in buffer and resets.
 */
void vUsbReceive(void *argument) {
  const TickType_t timeout = pdMS_TO_TICKS(20);
  uint8_t rxData;

  for (;;) {
    // Read byte from UART Rx buffer, skip loop if empty
    if (!xStreamBufferReceive(xUsbRxBuff, (void *) &rxData, 1, timeout))
      continue;

    // Send byte back for display
    usb.send(&usb, rxData);

    // Process command and reset buffer on <Enter> input
    if (rxData == CARRIAGE_RETURN) {
			usb.print(&usb, "\n");           	// Send newline back for display
      usbRxBuff[usbRxBuffIdx-1] = '\0'; // Replace carriage return with null terminator
			taskENTER_CRITICAL();
      usbCommandParse(usbRxBuff);    	 	// Parse and execute command
      taskEXIT_CRITICAL();
			usbRxBuffIdx = 0;               	// Reset buffer
    } 
		
		// Clear terminal on <Ctrl-c> input
		else if (rxData == SIGINT) {
			usbCommandParse((uint8_t *) "clear");    	 
			usbRxBuffIdx = 0;
		}
		
		// Erase character and move cursor backwards on <BS> input
		else if (rxData == BACKSPACE) {
			usb.print(&usb, " \b");
			if (usbRxBuffIdx)
				usbRxBuffIdx -= 2;
		}
  }
}

/**
 * @brief Interrupt handler for USB UART receive
 *
 * @details Circular append received byte to buffer and send byte to stream buffer
 * for processing in the Rx task.
 */
void USART6_IRQHandler() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  uint8_t rxData                      = usb.receive(&usb);
  usbRxBuff[usbRxBuffIdx++]           = rxData;
  usbRxBuffIdx %= MSG_BUFF_SIZE;

  xStreamBufferSendFromISR(xUsbRxBuff, (void *)&rxData, 1, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ===================================================================== *
 *                  HIGH RESOLUTION DATA ACQUISITION                     *
 * ===================================================================== */

void vDataAcquisitionH(void *argument) {
  float dt = 0.002;

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500Hz
  const TickType_t blockTime  = pdMS_TO_TICKS(0);
	

  for (;;) {
    // Block until 2ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Select which accelerometer to use
    pAccel_s = (pAccel_s->accelData[ZINDEX] < 15) ? &lAccel_s : &hAccel_s;
   
		#ifdef DUMMY
			const unsigned long accelX_length = 0x00007568;	// Load bearing definition???
		
		  /* 
		   * Update sensor data with dummy values 
		   * These arrays are defined in the files under /Data and are generated from 
		   * past flight data binaries with srec_cat.
		   */
			if(hDummyIdx < ACCELX_LENGTH - 1) {
				// Shift in floating point values and add to processed accelerometer array
				uint32_t tempX = (uint32_t)accelX[hDummyIdx+1] << 16 | accelX[hDummyIdx];
				uint32_t tempY = (uint32_t)accelY[hDummyIdx+1] << 16 | accelY[hDummyIdx];
				uint32_t tempZ = (uint32_t)accelZ[hDummyIdx+1] << 16 | accelZ[hDummyIdx];
				memcpy(&pAccel_s->accelData[0], &tempX, sizeof(float));
				memcpy(&pAccel_s->accelData[1], &tempY, sizeof(float));
				memcpy(&pAccel_s->accelData[2], &tempZ, sizeof(float));
				
				// Back convert to raw data 
				uint16_t xRaw = (short)(pAccel_s->accelData[0] / pAccel_s->sensitivity);
				uint16_t yRaw = (short)(pAccel_s->accelData[1] / pAccel_s->sensitivity);
				uint16_t zRaw = (short)(pAccel_s->accelData[2] / pAccel_s->sensitivity);
				pAccel_s->rawAccelData[0] = xRaw >> 8;
				pAccel_s->rawAccelData[1] = xRaw;
				pAccel_s->rawAccelData[2] = yRaw >> 8;
				pAccel_s->rawAccelData[3] = yRaw;
				pAccel_s->rawAccelData[4] = zRaw >> 8;
				pAccel_s->rawAccelData[5] = zRaw;
				
				// Shift in floating point values and add to processed gyroscope array
				tempX = (uint32_t)gyroX[hDummyIdx+1] << 16 | gyroX[hDummyIdx];
				tempY = (uint32_t)gyroY[hDummyIdx+1] << 16 | gyroY[hDummyIdx];
				tempZ = (uint32_t)gyroZ[hDummyIdx+1] << 16 | gyroZ[hDummyIdx];
				memcpy(&gyro_s.gyroData[0], &tempX, sizeof(float));
				memcpy(&gyro_s.gyroData[1], &tempY, sizeof(float));
				memcpy(&gyro_s.gyroData[2], &tempZ, sizeof(float));
				
				// Back convert to raw data 
				xRaw = (short)(gyro_s.gyroData[0]/gyro_s.sensitivity);
				yRaw = (short)(gyro_s.gyroData[1]/gyro_s.sensitivity);
				zRaw = (short)(gyro_s.gyroData[2]/gyro_s.sensitivity);
				gyro_s.rawGyroData[0] = xRaw >> 8;
				gyro_s.rawGyroData[1] = xRaw;
				gyro_s.rawGyroData[2] = yRaw >> 8;
				gyro_s.rawGyroData[3] = yRaw;
				gyro_s.rawGyroData[4] = zRaw >> 8;
				gyro_s.rawGyroData[5] = zRaw;

				hDummyIdx += 2;
			}
		#else
			lAccel_s.update(&lAccel_s);
			hAccel_s.update(&hAccel_s);
			gyro_s.update(&gyro_s);
		#endif
		
    // Add sensor data to dataframe
    mem.append(&mem, HEADER_HIGHRES);
    mem.appendBytes(&mem, pAccel_s->rawAccelData, KX134_1211_DATA_TOTAL);
    mem.appendBytes(&mem, gyro_s.rawGyroData, A3G4250D_DATA_TOTAL);
		
    // Only run calculations when enabled
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, GROUP_TASK_ENABLE_HIGHRES, pdFALSE, pdFALSE, blockTime);
    if (uxBits & GROUP_TASK_ENABLE_HIGHRES) {
      // Integrate attitude quaternion from rotations
      Quaternion qDot;
      Quaternion_init(&qDot);
      qDot.fromEuler(
          &qDot,
          (float)(dt * gyro_s.gyroData[ROLL_INDEX]),
          (float)(dt * gyro_s.gyroData[PITCH_INDEX]),
          (float)(dt * gyro_s.gyroData[YAW_INDEX])
      );
      qRot = Quaternion_mul(&qRot, &qDot);
      qRot.normalise(&qRot); // New attitude quaternion

      // Apply rotation to z-axis unit vector
      qRot.fRotateVector3D(&qRot, zUnit, vAttitude);

      // Calculate tilt angle
      // tilt = cos^-1(attitude · initial)
      cosine = zUnit[0] * vAttitude[0] + zUnit[1] * vAttitude[1] + zUnit[2] * vAttitude[2];
      tilt   = acos(cosine) * 180 / M_PI;
    }
  }
}

/* ===================================================================== *
 *                    LOW RESOLUTION DATA ACQUISITION                    *
 * ===================================================================== */

void vDataAcquisitionL(void *argument) {
  float dt = 0.020;
  KalmanFilter kf;
  KalmanFilter_init(&kf);

  // Initialise filter parameters
  float A[9] = {
      1.0, dt, 0.5 * (dt * dt),
      0.0, 1.0, dt,
      0.0, 0.0, 1.0
  };
  kf.A.pData = A;
  float Q[9] = {
      99.52, 0.0, 0.0,
      0.0, 1.42, 0.0,
      0.0, 0.0, 6.27
  };
  kf.Q.pData = Q;
  float R[4] = {
      97.92, 0.0,
      0.0, 0.61
  };
  kf.R.pData = R;
  float P[9] = {
      1, 0.0, 0.0,
      0.0, 0.1, 0.0,
      0.0, 0.0, 100.0
  };
  kf.P.pData = P;

  // Initialise measurement matrix
  arm_matrix_instance_f32 z;
  float zData[2] = {0.0, 0.0};
  arm_mat_init_f32(&z, 2, 1, zData);

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
  const TickType_t blockTime  = pdMS_TO_TICKS(0);

  for (;;) {
    // Block until 20ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Update baro data
		#ifdef DUMMY
			const unsigned long press_length = 0x00003A5C;
			if(lDummyIdx < PRESS_LENGTH - 1) {
				uint32_t tempPress = (uint32_t)press[lDummyIdx+1] << 16 | press[lDummyIdx];
				memcpy(&baro_s.press, &tempPress, sizeof(float));
				lDummyIdx += 2;
			}
		#else
			baro_s.update(&baro_s);
		#endif

    // Calculate altitude
    altitude = 44330 * (1.0 - pow(baro_s.press / baro_s.groundPress, 0.1903));

    // Add sensor data and barometer data to dataframe
    mem.append(&mem, HEADER_LOWRES);
    mem.appendBytes(&mem, baro_s.rawTemp, BMP581_DATA_SIZE);
    mem.appendBytes(&mem, baro_s.rawPress, BMP581_DATA_SIZE);

    // Only run calculations when enabled
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, GROUP_TASK_ENABLE_LOWRES, pdFALSE, pdFALSE, blockTime);
    if (uxBits & GROUP_TASK_ENABLE_LOWRES) {
      // Calculate state
      z.pData[0] = altitude;
      z.pData[1] = (cosine * 9.81 * pAccel_s->accelData[ZINDEX] - 9.81); // Acceleration measured in m/s^2
      kf.update(&kf, &z);
      velocity = kf.x.pData[1];
			
			avgPress.append(&avgPress, baro_s.press);
			avgVel.append(&avgVel, velocity);
    }
  }
}

void configure_interrupts() {
  __disable_irq();
  NVIC_SetPriority(EXTI1_IRQn, 9);
  NVIC_EnableIRQ(EXTI1_IRQn);
  NVIC_SetPriority(USART6_IRQn, 10);
  NVIC_EnableIRQ(USART6_IRQn);
  EXTI->RTSR |= 0X2;
  EXTI->IMR |= 0x2;
  SYSCFG->EXTICR[0] &= (~(0XF0));
  SYSCFG->EXTICR[0] = 0x30;
  __enable_irq();
}

// Unsure of actual fix for linker error
// temporary (lol) solution
void _init() {}
