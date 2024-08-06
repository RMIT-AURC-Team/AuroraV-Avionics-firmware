/**
 * @author Matt Ricci
 * @file main.c
 **/

#include "main.h"

// Task Handles
TaskHandle_t xDataAqcquisitionHHandle = NULL;
TaskHandle_t xDataAqcquisitionLHandle = NULL;
TaskHandle_t xFlashBufferHandle       = NULL;
TaskHandle_t xStateUpdateHandle       = NULL;
TaskHandle_t xLoRaTransmitHandle      = NULL;
TaskHandle_t xUsbTransmitHandle       = NULL;

EventGroupHandle_t xTaskEnableGroup; // 0: FLASH,  1: HIGHRES, 2: LOWRES, 7: IDLE

KX134_1211 lAccel_s;
KX134_1211 hAccel_s;
KX134_1211 accel_s;
A3G4250D gyro_s;
BMP581 baro_s;

UART usb;
LoRa lora;
Flash flash;

MessageBufferHandle_t xUsbRxBuff;
MessageBufferHandle_t xUsbTxBuff;
const size_t xUsbBuffSize = 128;

MessageBufferHandle_t xLoRaRxBuff;
MessageBufferHandle_t xLoRaTxBuff;
const size_t xLoRaBuffSize = 128;

#define BUFF_SIZE 21000 // 2s worth of data in buffer
#define PAGE_SIZE 256
MemBuff mem;
uint8_t buff[BUFF_SIZE];
uint8_t outBuff[PAGE_SIZE];

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

  // Initialise timers
  TIM6init();
  TIM7init();

  // Configure peripherals
  CANGPIO_config();
  CAN_Peripheral_config();

  Flash_init(&flash, FLASH_PORT, FLASH_CS);
  UART_init(&usb, USB_INTERFACE, USB_PORT, USB_BAUD, OVER8);

  LoRa_init(&lora, LORA_PORT, LORA_CS, BW250, SF9, CR5);
  xLoRaTxBuff = xMessageBufferCreate(xLoRaBuffSize);

  // Initialise sensors
  BMP581_init(&baro_s, BARO_PORT, BARO_CS, BMP581_TEMP_SENSITIVITY, BMP581_PRESS_SENSITIVITY);
  A3G4250D_init(&gyro_s, GYRO_PORT, GYRO_CS, A3G4250D_SENSITIVITY, GYRO_AXES, GYRO_SIGN);
  KX134_1211_init(&lAccel_s, ACCEL_PORT_1, ACCEL_CS_1, ACCEL_SCALE_LOW, ACCEL_AXES_1, ACCEL_SIGN_1);
  KX134_1211_init(&hAccel_s, ACCEL_PORT_2, ACCEL_CS_2, ACCEL_SCALE_HIGH, ACCEL_AXES_2, ACCEL_SIGN_2);
  accel_s              = lAccel_s;

  unsigned int CANHigh = 0;
  unsigned int CANLow  = 0;
  unsigned int id      = 0x603;
  CAN_TX(1, 8, CANHigh, CANLow, id);

  MemBuff_init(&mem, buff, BUFF_SIZE, PAGE_SIZE);
  Quaternion_init(&qRot);

  xTaskEnableGroup = xEventGroupCreate();
  xEventGroupClearBits(xTaskEnableGroup, 0xFF);

  // Create task handles
  xTaskCreate(vDataAcquisitionH, "HDataAcq", 256, NULL, configMAX_PRIORITIES - 2, &xDataAqcquisitionHHandle);
  xTaskCreate(vDataAcquisitionL, "LDataAcq", 256, NULL, configMAX_PRIORITIES - 3, &xDataAqcquisitionLHandle);
  xTaskCreate(vStateUpdate, "StateUpdate", 256, NULL, configMAX_PRIORITIES - 4, &xStateUpdateHandle);
  xTaskCreate(vFlashBuffer, "FlashData", 256, NULL, configMAX_PRIORITIES - 1, &xFlashBufferHandle);
  xTaskCreate(vLoRaTransmit, "LoRaTx", 256, NULL, configMAX_PRIORITIES - 5, &xLoRaTransmitHandle);
  // xTaskCreate(vUsbTransmit, "USB Rx", 256, NULL, configMAX_PRIORITIES - 6, &xUsbTransmitHandle);

  vTaskStartScheduler();

  // Should never reach here
  while (1) {
  }
}

/* ===================================================================== *
 *                            STATE MANAGEMENT                           *
 * ===================================================================== */

void vStateUpdate(void *argument) {
  unsigned int ms = 0;
  union {
    unsigned int i;
    uint8_t a[4];
  } u;
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
      id      = 0x602;
      CAN_TX(1, 8, CANHigh, CANLow, id);
    }

    switch (currentState) {
    case PRELAUNCH:
      if (accel_s.accelData[ZINDEX] >= ACCEL_LAUNCH) {
        xEventGroupSetBits(xTaskEnableGroup, 0x80); // Enable flash
        xEventGroupSetBits(xTaskEnableGroup, 0x06); // Enable data acquisition
        currentState = LAUNCH;
        // Add launch event dataframe to buffer
        mem.append(&mem, HEADER_EVENT_LAUNCH);
        mem.appendBytes(&mem, u.a, 4);
      }
      break;
    case LAUNCH:
      // Send altitude to aerobrakes via CAN
      CANHigh = 0x00000000;
      CANLow  = (unsigned int)altitude;
      id      = 0x601;
      CAN_TX(1, 8, CANHigh, CANLow, id);
      // Transition to motor burnout state on velocity decrease
      if (true) { // TODO: Change to decreasing velocity
        currentState = COAST;
        // Add motor burnout event dataframe to buffer
        mem.append(&mem, HEADER_EVENT_COAST);
        mem.appendBytes(&mem, u.a, 4);
      }
      break;
    case COAST:
      // Send altitude to aerobrakes via CAN
      CANHigh = 0x00000000;
      CANLow  = (unsigned int)altitude;
      id      = 0x601;
      CAN_TX(1, 8, CANHigh, CANLow, id);
      // Transition to apogee state on three way vote of altitude, velocity, and tilt
      // apogee is determined as two of three conditions evaluating true
      if (((true) + (tilt >= 90) + (velocity < 0.0f)) >= 2) { // TODO: change first condition to decreasing altitude
        currentState = APOGEE;
        // Add apogee event dataframe to buffer
        mem.append(&mem, HEADER_EVENT_APOGEE);
        mem.appendBytes(&mem, u.a, 4);
        // Send transmission to trigger apogee E-matches
      }
      break;
    case APOGEE:
      if (isAltitude1300ft(altitude)) {
        currentState = DESCENT;
        // Enable descent state
        mem.append(&mem, HEADER_EVENT_DESCENT);
        mem.appendBytes(&mem, u.a, 4);
        // Add descent event dataframe to buffer
      }
      break;
    case DESCENT:
      // Handle descent state actions
      break;
    }
		
		if(currentState >= LAUNCH) {
			LoRa_Packet packet = LoRa_AVD1(LORA_HEADER_AVD1, accel_s.rawAccelData, KX134_1211_DATA_TOTAL);
			xMessageBufferSend(xLoRaTxBuff, (void *) &packet, sizeof(packet), 0);
		}
		
    ms += 2;
    u.i = ms;
  }
}

/* ===================================================================== *
 *                            FLASH HANDLING                             *
 * ===================================================================== */

// Use idle time to write flash
void vApplicationIdleHook(void) {
  // Write if a page is available in the buffer
  if (currentState == LAUNCH && mem.pageReady)
    xEventGroupSetBits(xTaskEnableGroup, 0x01);
}

void vFlashBuffer(void *argument) {
  const TickType_t timeout = pdMS_TO_TICKS(1);
  uint32_t pageAddr        = 0;
  for (;;) {
    // Wait for write flag to be ready, clear flag on exit
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, 0x01, pdTRUE, pdFALSE, timeout);
    if ((uxBits & 0x01) == 0x01) {
      bool success = mem.readPage(&mem, outBuff); // Flush data to output buffer
      if (success) {
        // Write data to flash memory
        // Flash_Page_Program(flashAddress, outBuff, sizeof(outBuff));
        flash.writePage(&flash, pageAddr, outBuff);
        pageAddr += 0x100;
      }
    }
  }
}

/* ===================================================================== *
 *                             LORA HANDLING                             *
 * ===================================================================== */

void vLoRaTransmit(void *argument) {
  const TickType_t timeout = pdMS_TO_TICKS(500);
  uint8_t rxData[16];
  size_t xReceivedBytes;

  for (;;) {
    xReceivedBytes = xMessageBufferReceive(
        xLoRaTxBuff,
        (void *)rxData,
        sizeof(rxData),
        timeout
    );

    if (xReceivedBytes)
      lora.transmit(&lora, rxData);
  }
}

/* ===================================================================== *
 *                             UART HANDLING                             *
 * ===================================================================== */
void vUsbTransmit(void *argument) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(500); // 2Hz
  // Initialise message buffer
  xUsbTxBuff = xMessageBufferCreate(xUsbBuffSize);
  for (;;) {
  }
}

/* ===================================================================== *
 *                  HIGH RESOLUTION DATA ACQUISITION                     *
 * ===================================================================== */

void vDataAcquisitionH(void *argument) {
  float dt = 0.002;

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500Hz
  const TickType_t blockTime  = pdMS_TO_TICKS(0); // Don't need to block calculations
  for (;;) {
    // Block until 2ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Select which accelerometer to use
    accel_s = (accel_s.accelData[ZINDEX] < 15) ? lAccel_s : hAccel_s;

    // Update sensor data
    accel_s.update(&accel_s);
    gyro_s.update(&gyro_s);

    // Add sensor data to dataframe
    mem.append(&mem, HEADER_HIGHRES);
    mem.appendBytes(&mem, accel_s.rawAccelData, KX134_1211_DATA_TOTAL);
    mem.appendBytes(&mem, gyro_s.rawGyroData, A3G4250D_DATA_TOTAL);

    // Only run calculations when enabled
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, 0x02, pdFALSE, pdFALSE, blockTime);
    if ((uxBits & 0x02) == 0x02) {
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
  const TickType_t blockTime  = pdMS_TO_TICKS(0);  // Don't need to block

  for (;;) {
    // Block until 20ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Update baro data
    baro_s.update(&baro_s);

    // Calculate altitude
    altitude = 44330 * (1.0 - pow(baro_s.press / baro_s.groundPress, 0.1903));

    // Add sensor data and barometer data to dataframe
    mem.append(&mem, HEADER_LOWRES);
    mem.appendBytes(&mem, baro_s.rawTemp, 3);
    mem.appendBytes(&mem, baro_s.rawPress, 3);

    // Only run calculations when enabled
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, 0x04, pdFALSE, pdFALSE, blockTime);
    if ((uxBits & 0x04) == 0x04) {
      // Calculate state
      z.pData[0] = altitude;
      z.pData[1] = (cosine * 9.81 * accel_s.accelData[ZINDEX] - 9.81); // Acceleration measured in m/s^2
      kf.update(&kf, &z);
    }
  }
}

// Unsure of actual fix for linker error
// temporary (lol) solution
void _init() {}
