#include "main.h"

// Task Handles
TaskHandle_t xDataAqcquisitionHHandle = NULL;
TaskHandle_t xDataAqcquisitionLHandle = NULL;
TaskHandle_t xFlashBufferHandle       = NULL;
TaskHandle_t xStateUpdateHandle       = NULL;
EventGroupHandle_t xTaskEnableGroup; // 0: FLASH,  1: HIGHRES, 2: LOWRES, 7: IDLE

// Calculated attitude variables
Quaternion qRot;                // Global attitude quaternion
float vAttitude[3] = {0, 0, 1}; // Attitude vector
float zUnit[3]     = {0, 0, 1}; // Z unit vector
float cosine       = 0;         // Tilt angle cosine
float tilt         = 0;         // Tilt angle

// Flight dynamics state variables
float altitude = 0;                  // Current altitude
float velocity = 0;                  // Current vertical velocity
float accel    = 0;                  // Current body-axis acceleration

#define BUFF_SIZE 21000              // 2s worth of data in buffer
#define PAGE_SIZE 256
MemBuff mem;
uint8_t buff[BUFF_SIZE];
uint8_t outBuff[PAGE_SIZE];
uint8_t buffCount = 0;               // Test variable for reading output buffer

enum State currentState = PRELAUNCH; // Boot in prelaunch
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
void initHeartbeat();

int main(void) {
  //  configure_RCC_APB1();
  //  configure_RCC_APB2();
  //  configure_RCC_AHB1();
  SystemInit();
  initUART();
  initHeartbeat();

  MemBuff_init(&mem, buff, BUFF_SIZE, PAGE_SIZE);
  Quaternion_init(&qRot);

  xTaskEnableGroup = xEventGroupCreate();
  xEventGroupClearBits(xTaskEnableGroup, 0xFF);

  // Create task handles
  xTaskCreate(vDataAcquisitionH, "HDataAcq", 256, NULL, configMAX_PRIORITIES - 2, &xDataAqcquisitionHHandle);
  xTaskCreate(vDataAcquisitionL, "LDataAcq", 256, NULL, configMAX_PRIORITIES - 3, &xDataAqcquisitionLHandle);
  xTaskCreate(vStateUpdate, "StateUpdate", 256, NULL, configMAX_PRIORITIES - 4, &xStateUpdateHandle);
  xTaskCreate(vFlashBuffer, "FlashData", 256, NULL, configMAX_PRIORITIES - 1, &xFlashBufferHandle);

  vTaskStartScheduler();

  // Should never reach here
  while (1) {
  }
}

/* =====================================================================
 *                            FLASH HANDLING
 * ===================================================================== */

// Use idle time to write flash
void vApplicationIdleHook(void) {
  // Write if a page is available in the buffer
  if (currentState == LAUNCH && mem.pageReady)
    xEventGroupSetBits(xTaskEnableGroup, 0x01);
}

void vFlashBuffer(void *argument) {
  const TickType_t timeout = 10;
  for (;;) {
    // Wait for write flag to be ready, clear flag on exit
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, 0x01, pdTRUE, pdFALSE, timeout);
    if ((uxBits & 0x01) == 0x01) {
      _Bool success = mem.readPage(&mem, outBuff); // Flush data to output buffer
    }
  }
}

/* =====================================================================
 *                            STATE MANAGEMENT
 * ===================================================================== */

void vStateUpdate(void *argument) {
  int periods = 0;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
  //
  for (;;) {
    switch (currentState) {
    case PRELAUNCH:
      // if (periods >= 100 || isAccelerationAbove5Gs()) { // Using periods to simulate >2s timing
      if (isAccelerationAbove5Gs()) {
        xEventGroupSetBits(xTaskEnableGroup, 0x80); // Enable flash
        xEventGroupSetBits(xTaskEnableGroup, 0x06); // Enable data acquisition
        currentState = LAUNCH;
        // Add launch event dataframe to buffer
      }
      break;
    case LAUNCH:
      // Send velocity and altitude to aerobrakes via CAN
      sendVelocityAndAltitude();
      if (isVelocityDecreasing()) {
        currentState = MOTOR_BURNOUT;
        // Add motor burnout event dataframe to buffer
      }
      break;
    case MOTOR_BURNOUT:
      if (isAltitudeDropping() && isTiltAngleAbove90() && isNegativeVelocity()) {
        currentState = APOGEE;
        // Add apogee event dataframe to buffer
        // Send transmission to trigger apogee E-matches
      }
      break;
    case APOGEE:
      if (isAltitude1300ft()) {
        currentState = DESCENT;
        // Enable descent state
        // Add descent event dataframe to buffer
      }
      break;
    case DESCENT:
      // Handle descent state actions
      break;
    default:
      // Handle unexpected state
      break;
    }

    periods++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/* =====================================================================
 *                  HIGH RESOLUTION DATA ACQUISITION
 * ===================================================================== */

void vDataAcquisitionH(void *argument) {
  unsigned int index = 0;
  float dt           = 0.002;
  float roll, pitch, yaw;

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500Hz
  const TickType_t timeout    = pdMS_TO_TICKS(2); // Shouldn't block longer than period
  for (;;) {
    // Read sensor data
    memcpy(&accel, baccel + index, sizeof(accel));
    memcpy(&roll, gyroX + index, sizeof(roll));
    memcpy(&pitch, gyroY + index, sizeof(pitch));
    memcpy(&yaw, gyroZ + index, sizeof(yaw));

    // Add sensor data and quaternion to dataframe
    mem.append(&mem, HEADER_HIGHRES);
    mem.append(&mem, buffCount++); // Accel X high byte
    mem.append(&mem, buffCount++); // Accel X low byte
    mem.append(&mem, buffCount++); // Accel Y high byte
    mem.append(&mem, buffCount++); // Accel Y low byte
    mem.append(&mem, buffCount++); // Accel Z high byte
    mem.append(&mem, buffCount++); // Accel Z low byte
    mem.append(&mem, buffCount++); // Gyro X high byte
    mem.append(&mem, buffCount++); // Gyro X low byte
    mem.append(&mem, buffCount++); // Gyro Y high byte
    mem.append(&mem, buffCount++); // Gyro Y low byte
    mem.append(&mem, buffCount++); // Gyro Z high byte
    mem.append(&mem, buffCount++); // Gyro Z low byte
    mem.append(&mem, buffCount++); // Magnet X high byte
    mem.append(&mem, buffCount++); // Magnet X low byte
    mem.append(&mem, buffCount++); // Magnet Y high byte
    mem.append(&mem, buffCount++); // Magnet Y low byte
    mem.append(&mem, buffCount++); // Magnet Z high byte
    mem.append(&mem, buffCount++); // Magnet Z low byte

    // Only run calculations when enabled
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, 0x02, pdFALSE, pdFALSE, timeout);
    if ((uxBits & 0x02) == 0x02) {
      // Integrate attitude quaternion from rotations
      Quaternion qDot;
      Quaternion_init(&qDot);
      qDot.fromEuler(
          &qDot,
          (float)(dt * roll),
          (float)(dt * pitch),
          (float)(dt * yaw)
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

    index = (index + 4) % accel_length; // Handle buffer overflow;
    // Heartbeat
    if (index % 160 == 0)
      GPIOB->ODR ^= GPIO_ODR_OD14;

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/* =====================================================================
 *                    LOW RESOLUTION DATA ACQUISITION
 * ===================================================================== */

void vDataAcquisitionL(void *argument) {
  unsigned int index = 0;
  float dt           = 0.020;

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
  const TickType_t timeout    = pdMS_TO_TICKS(20); // Shouldn't block longer than period
  for (;;) {
    // Read sensor data
    memcpy(&altitude, baro + index, sizeof(altitude));

    // Add sensor data and quaternion to dataframe
    mem.append(&mem, HEADER_LOWRES);
    mem.append(&mem, buffCount++);
    mem.append(&mem, buffCount++);
    mem.append(&mem, buffCount++);
    mem.append(&mem, buffCount++);
    mem.append(&mem, buffCount++);
    mem.append(&mem, buffCount++);
    mem.append(&mem, buffCount++);
    mem.append(&mem, buffCount++);

    // Only run calculations when enabled
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, 0x04, pdFALSE, pdFALSE, timeout);
    if ((uxBits & 0x04) == 0x04) {
      // Calculate state
      z.pData[0] = altitude;
      z.pData[1] = (cosine * 9.81 * accel - 9.81); // Acceleration measured in m/s^2
      kf.update(&kf, &z);

      union {
        float f;
        uint8_t a[4];
      } u;
      u.f = kf.x.pData[1];
    }

    index = (index + 4) % baro_length; // Handle buffer overflow;
    // Heartbeat
    if (index % 16 == 0)
      GPIOB->ODR ^= GPIO_ODR_OD7;

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void initHeartbeat() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  __ASM("NOP");
  __ASM("NOP");

  GPIOB->MODER   &= (~(GPIO_MODER_MODE7_Msk));
  GPIOB->MODER   |= ((0x1 << GPIO_MODER_MODE7_Pos));
  GPIOB->OTYPER  &= (uint16_t)(~(GPIO_OTYPER_OT7));
  GPIOB->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED7_Msk));
  GPIOB->OSPEEDR |= ((0x2 << GPIO_OSPEEDR_OSPEED7_Pos));
  GPIOB->ODR     &= ~GPIO_ODR_OD7;

  GPIOB->MODER   &= (~(GPIO_MODER_MODE14_Msk));
  GPIOB->MODER   |= ((0x1 << GPIO_MODER_MODE14_Pos));
  GPIOB->OTYPER  &= (uint16_t)(~(GPIO_OTYPER_OT14));
  GPIOB->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED14_Msk));
  GPIOB->OSPEEDR |= ((0x2 << GPIO_OSPEEDR_OSPEED14_Pos));
  GPIOB->ODR     &= ~GPIO_ODR_OD14;
}

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
  xTaskHandle *bad_task_handle = pxTask;
  signed char *bad_task_name   = pcTaskName;
  for (;;);
}

// Unsure of actual fix for linker error
// temporary (lol) solution
void _init() {}
