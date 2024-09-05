/***********************************************************************************
 * @file        main.c                                                             *
 * @author      Matt Ricci                                                         *
 * @brief       Main application entry point and system initialization.            *
 *                                                                                 *
 * @todo Implement globals as context struct to pass to external functions.        *
 *       e.g. passing context of flash, uart, etc. to control functions.           *
 *                                                                                 *
 * @todo Implement startup task to isolate initialisations from main.              *
 ***********************************************************************************/

#include "main.h"

long hDummyIdx = 0;
long lDummyIdx = 0;

// RTOS event groups
EventGroupHandle_t xTaskEnableGroup; // 0: FLASH,  1: HIGHRES, 2: LOWRES, 3: LORA, 7: IDLE
EventGroupHandle_t xMsgReadyGroup;   // 0: LORA, 1: USB

// RTOS message buffers
MessageBufferHandle_t xLoRaTxBuff;
MessageBufferHandle_t xUsbTxBuff;
StreamBufferHandle_t xUsbRxBuff;

SemaphoreHandle_t xUsbMutex;

/* =============================================================================== */
/**
 * @brief Main application entry point.
 *
 * Initializes microcontroller peripherals, creates the system initialization task,
 * and starts the FreeRTOS scheduler.
 *
 * @return int  Exit status (should never return)
 * =============================================================================== */

int main(void) {
  // Initialise clock sources and peripheral busses
  configure_RCC_APB1();
  configure_RCC_APB2();
  configure_RCC_AHB1();

  // Initialize GPIO pins for peripherals
  configure_MISC_GPIO();
  configure_UART3_GPS();
  configure_SPI1_Sensor_Suite();
  configure_SPI3_LoRa();
  configure_SPI4_Flash();
  configure_interrupts();

  // Initialise timers
  TIM6init();
  TIM7init();

  // Configure CAN
  CANGPIO_config();
  CAN_Peripheral_config();

#ifdef TRACE
  xTraceEnable(TRC_START);
#endif

#ifdef FLIGHT_TEST
  GPIOB->ODR ^= 0x8000;
  GPIOD->ODR ^= 0x8000;
#endif

  // Send AB ground test message over CAN
  unsigned int CANHigh = 0;
  unsigned int CANLow  = 0;
  unsigned int id      = 0x603;
  CAN_TX(2, 8, CANHigh, CANLow, id);

  // Create and start the system initialization task
  TaskHandle_t xSystemInitHandle;
  xTaskCreate(vSystemInit, "SystemInit", 8192, NULL, configMAX_PRIORITIES - 1, &xSystemInitHandle);
  vTaskStartScheduler();

  // The scheduler should never return
  return 0;
}

/* =============================================================================== */
/**
 * @brief Initialisation task for drivers and other RTOS tasks
 *
 * Performs initial setup for various peripherals, ensuring all components
 * are ready for data acquisition and system state management. This task also
 * initializes RTOS event groups and message buffers to manage inter-task
 * communication.
 *
 * @return void
 * =============================================================================== */

void vSystemInit(void *argument) {

  // Allow a second for external devices to finish startup sequences
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Create event groups for task synchronization and message signaling
  xTaskEnableGroup = xEventGroupCreate(); // 0: FLASH,  1: HIGHRES, 2: LOWRES, 3: LORA, 7: IDLE
  xMsgReadyGroup   = xEventGroupCreate();
  xEventGroupSetBits(xMsgReadyGroup, GROUP_MESSAGE_READY_LORA);

  /* ----------------------------------- Flash Initialization ------------------------------------ */

  // Initialise circular memory buffer
  MemBuff mem;
  uint8_t buff[MEM_BUFF_SIZE];
  MemBuff_init(&mem, buff, MEM_BUFF_SIZE, FLASH_PAGE_SIZE);

  // Initialise SPI flash driver
  Flash flash;
  Flash_init(&flash, FLASH_PORT, FLASH_CS, FLASH_PAGE_SIZE, FLASH_PAGE_COUNT);

  /* ------------------------------- Communication Initialization -------------------------------- */

  // Initialise USB UART driver
  UART usb;
  xUsbTxBuff = xMessageBufferCreate(USB_TX_SIZE);
  xUsbRxBuff = xStreamBufferCreate(USB_RX_SIZE, 1);
  xUsbMutex  = xSemaphoreCreateMutex();
  UART_init(&usb, USB_INTERFACE, USB_PORT, USB_BAUD, OVER8);

  // Initialise USB shell driver
  Shell shell;
  Shell_init(&shell, usb, flash);

  // Initialise LoRa driver
  LoRa lora;
  xLoRaTxBuff = xMessageBufferCreate(LORA_BUFF_SIZE);
  LoRa_init(&lora, LORA_PORT, LORA_CS, BW500, SF9, CR5);

  /* ---------------------------------- Sensor Initialization ----------------------------------- */

  // Initialise accelerometer drivers
  static KX134_1211 lAccel, hAccel, *accel;
  KX134_1211_init(&lAccel, ACCEL_PORT_1, ACCEL_CS_1, ACCEL_SCALE_LOW, ACCEL_AXES_1, ACCEL_SIGN_1);
  KX134_1211_init(&hAccel, ACCEL_PORT_2, ACCEL_CS_2, ACCEL_SCALE_HIGH, ACCEL_AXES_2, ACCEL_SIGN_2);
  accel = &lAccel;

  // Initialise gyroscope driver
  static A3G4250D gyro;
  A3G4250D_init(&gyro, GYRO_PORT, GYRO_CS, A3G4250D_SENSITIVITY, GYRO_AXES, GYRO_SIGN);

  // Initialise barometer driver
  static BMP581 baro;
  BMP581_init(&baro, BARO_PORT, BARO_CS, BMP581_TEMP_SENSITIVITY, BMP581_PRESS_SENSITIVITY);

  /* ---------------------------------- State Initialization ------------------------------------ */

  // Initialize system state structure
  static ctxState state;
  state.currentState = PRELAUNCH;
  state.cosine       = 0;
  state.tilt         = 0;
  state.altitude     = 0;
  state.velocity     = 0;
  memcpy(&state.vAttitude, (float[3]){0, 0, 1}, 3);
  memcpy(&state.zUnit, (float[3]){0, 0, 1}, 3);
  Quaternion_init(&state.qRot);

  // Initialize sliding window filters for pressure and velocity
  float avgPressBuff[AVG_BUFF_SIZE];
  SlidingWindow_init(&state.avgPress, avgPressBuff, AVG_BUFF_SIZE);

  float avgVelBuff[AVG_BUFF_SIZE];
  SlidingWindow_init(&state.avgVel, avgVelBuff, AVG_BUFF_SIZE);

  /*********************************************************************************************************************
   *                                                 TASK INIT                                                         *
   *********************************************************************************************************************/

  static Handles handles;

  /* ------------------------------------- High Resolution Data Acquisition -------------------------------------------*/

  // Create high-resolution data acquisition task
  static ctxHDataAcquisition hDataAcq;
  hDataAcq.state            = state;
  hDataAcq.mem              = mem;
  hDataAcq.xTaskEnableGroup = xTaskEnableGroup;
  hDataAcq.xUsbMutex        = xUsbMutex;
  hDataAcq.xUsbTxBuff       = xUsbTxBuff;
  hDataAcq.gyro             = gyro;
  hDataAcq.hAccel           = hAccel;
  hDataAcq.lAccel           = lAccel;
  hDataAcq.accel            = accel;
  xTaskCreate(vHDataAcquisition, "HDataAcq", 512, &hDataAcq, configMAX_PRIORITIES - 2, &handles.xHDataAcquisitionHandle);

  /* ------------------------------------- Low Resolution Data Acquisition ---------------------------------------------*/

  // Create low-resolution data acquisition task
  static ctxLDataAcquisition lDataAcq;
  lDataAcq.state            = state;
  lDataAcq.mem              = mem;
  lDataAcq.xTaskEnableGroup = xTaskEnableGroup;
  lDataAcq.xUsbMutex        = xUsbMutex;
  lDataAcq.xUsbTxBuff       = xUsbTxBuff;
  lDataAcq.baro             = baro;
  lDataAcq.accel            = accel;
  xTaskCreate(vLDataAcquisition, "LDataAcq", 512, &lDataAcq, configMAX_PRIORITIES - 3, &handles.xLDataAcquisitionHandle);

  /* ----------------------------------------------- State Update ------------------------------------------------------*/

  // Create state update task
  static ctxFlightState flightState;
  flightState.state            = state;
  flightState.handles          = handles;
  flightState.xTaskEnableGroup = xTaskEnableGroup;
  flightState.xUsbMutex        = xUsbMutex;
  flightState.xUsbTxBuff       = xUsbTxBuff;
  flightState.accel            = accel;
  xTaskCreate(vStateUpdate, "StateUpdate", 128, &flightState, configMAX_PRIORITIES - 4, &handles.xStateUpdateHandle);

  /* ------------------------------------------------ Flash Write-------------------------------------------------------*/

  // Create idle task (responsible for enabling flash operations)
  static ctxIdle idle;
  idle.currentState     = &state.currentState;
  idle.mem              = mem;
  idle.xTaskEnableGroup = xTaskEnableGroup;
  xTaskCreate(vIdle, "Idle", 128, &idle, tskIDLE_PRIORITY, &handles.xIdleHandle);

  // Create flash write task
  static ctxFlashBuffer flashBuffer;
  flashBuffer.currentState     = &state.currentState;
  flashBuffer.mem              = mem;
  flashBuffer.flash            = flash;
  flashBuffer.xTaskEnableGroup = xTaskEnableGroup;
  xTaskCreate(vFlashBuffer, "FlashData", 128, &flashBuffer, configMAX_PRIORITIES - 1, &handles.xFlashBufferHandle);

  /* --------------------------------------------  LoRa Communication ---------------------------------------------------*/

  // Create LoRa sample collection task
  static ctxLoRaSample loraSample;
  loraSample.state  = state;
  loraSample.hAccel = hAccel;
  loraSample.lAccel = lAccel;
  loraSample.gyro   = gyro;
  xTaskCreate(vLoRaSample, "LoRaSample", 128, &loraSample, configMAX_PRIORITIES - 6, &handles.xLoRaSampleHandle);

  // Create LoRa Tx task
  static ctxLoRaTransmit loraTransmit;
  loraTransmit.lora = lora;
  xTaskCreate(vLoRaTransmit, "LoRaTx", 128, &loraTransmit, configMAX_PRIORITIES - 5, &handles.xLoRaTransmitHandle);

  /* ---------------------------------------------- USB Communication ---------------------------------------------------*/

  // Create USB Tx task
  static ctxUsbTransmit usbTransmit;
  usbTransmit.usb = usb;
  xTaskCreate(vUsbTransmit, "UsbTx", 256, &usbTransmit, configMAX_PRIORITIES - 6, &handles.xUsbTransmitHandle);

  // Create USB Rx task
  static ctxUsbReceive usbReceive;
  usbReceive.usb   = usb;
  usbReceive.shell = shell;
  xTaskCreate(vUsbReceive, "UsbRx", 256, &usbReceive, configMAX_PRIORITIES - 6, &handles.xUsbReceiveHandle);

  /* ----------------------------------------------- GPS Acquisition ----------------------------------------------------*/

  // Create GPS data reading and processing task
  static ctxGpsTransmit gpsTransmit;
  gpsTransmit.currentState = &state.currentState;
  xTaskCreate(vGpsTransmit, "GpsRead", 512, &gpsTransmit, configMAX_PRIORITIES - 6, &handles.xGpsTransmitHandle);

  // Suspend the system initialization task (it only needs to run once)
  vTaskSuspend(NULL);
}

/**
 * @todo Refactor and document
 */
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
