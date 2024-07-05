#include "main.h"

// Task Handles
TaskHandle_t xDataAqcquisitionHHandle = NULL;
TaskHandle_t xDataAqcquisitionLHandle = NULL;
TaskHandle_t xFlashBufferHandle       = NULL;
TaskHandle_t xStateUpdateHandle       = NULL;
TaskHandle_t xLoRaCommunicateHandle	  = NULL;
EventGroupHandle_t xTaskEnableGroup; // 0: FLASH,  1: HIGHRES, 2: LOWRES, 7: IDLE

// Calculated attitude variables
Quaternion qRot;                // Global attitude quaternion
float vAttitude[3] = {0, 0, 1}; // Attitude vector
float zUnit[3]     = {0, 0, 1}; // Z unit vector
float cosine       = 0;         // Tilt angle cosine
float tilt         = 0;         // Tilt angle

// Angular rotations
float roll  = 0;
float pitch = 0;
float yaw   = 0;

// Flight dynamics state variables
float altitude = 0;     // Current altitude
float velocity = 0;     // Current vertical velocity
float accelZ   = 0;     // Current body-axis acceleration

uint8_t accel[6] = {0, 0, 0, 0, 0, 0};
uint8_t gyro[6];
uint8_t magnet[6];

#define BUFF_SIZE 21000 // 2s worth of data in buffer
#define PAGE_SIZE 256
MemBuff mem;
uint8_t buff[BUFF_SIZE];
uint8_t outBuff[PAGE_SIZE];
uint8_t buffCount = 0;               // Test variable for reading output buffer

enum State currentState = PRELAUNCH; // Boot in prelaunch
int interval2ms         = 0;

const struct LoRa_Registers LoRa_Regs = {0, 1, 0xd, 0xE, 0xF, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0X1C, 0X1D, 0X1E, 0X1F, 0X20, 0X21, 0X22, 0X23, 0X24, 0X25, 0X28, 0X29, 0X2A, 0X2C, 0X31, 0X33, 0X37, 0X39, 0X3B, 0X3D, 0X40, 0X41};

int main(void) {
  SystemInit();

  // Bring up RCC
  configure_RCC_APB1();
  configure_RCC_APB2();
  configure_RCC_AHB1();
  configure_MISC_GPIO();
  configure_UART3_GPS();
  configure_UART6_Serial_2_mini_USB();
  configure_SPI1_Sensor_Suite();
  configure_SPI2_Flash();
  configure_SPI3_LoRa();

  // Initialise timers
  TIM6init();
  TIM7init();

  // Configure peripherals
  CANGPIO_config();
  CAN_Peripheral_config();
  configure_LoRa_module();

  // Configure accelerometer
  write_ACCELL_1(ACCEL_CNTL1, 0x50);                                     // Accel select, 32g sensitivity
  uint8_t ODCNTL = read_ACCELL_1(ACCEL_ODCNTL);                          // Read from register for reserve mask
  write_ACCELL_1(ACCEL_ODCNTL, (ACCEL_ODCNTL_RESERVED & ODCNTL) | 0x2A); // No filter, fast startup, 800Hz
  write_ACCELL_1(ACCEL_CNTL1, 0xD0);                                     // Enable PC1

  // Configure gyroscope
  write_GYRO(GYRO_CTRL_REG1, GYRO_CTRL_REG1_ODR_800Hz | GYRO_CTRL_REG1_AXIS_ENABLE | GYRO_CTRL_REG1_PD_ENABLE);
	//write_GYRO(GYRO_CTRL_REG1, 0xCF);
	
	
  // Configure magnetometer
  write_MAG(MAGNET_CTRL_REG1, MAGNET_CTRL_REG1_FAST);
  write_MAG(MAGNET_CTRL_REG2, MAGNET_CTRL_REG2_FS16);

  // Configure barometer
  write_BARO(BARO_ODR_CFG, BARO_ODR_CFG_PWR | BARO_ODR_CFG_DEEP_DIS);
  uint8_t OSRCFG = read_BARO(BARO_OSR_CFG);
  write_BARO(BARO_OSR_CFG, (BARO_OSR_CFG_RESERVED * OSRCFG) | BARO_OSR_CFG_PRESS_EN);

  MemBuff_init(&mem, buff, BUFF_SIZE, PAGE_SIZE);
  Quaternion_init(&qRot);

  xTaskEnableGroup = xEventGroupCreate();
  xEventGroupClearBits(xTaskEnableGroup, 0xFF);

  // Create task handles
  xTaskCreate(vDataAcquisitionH, "HDataAcq", 256, NULL, configMAX_PRIORITIES - 2, &xDataAqcquisitionHHandle);
  xTaskCreate(vDataAcquisitionL, "LDataAcq", 256, NULL, configMAX_PRIORITIES - 3, &xDataAqcquisitionLHandle);
  xTaskCreate(vStateUpdate, "StateUpdate", 256, NULL, configMAX_PRIORITIES - 4, &xStateUpdateHandle);
  xTaskCreate(vFlashBuffer, "FlashData", 256, NULL, configMAX_PRIORITIES - 1, &xFlashBufferHandle);
  //xTaskCreate(vLoRaCommunicate, "LoRa", 256, NULL, configMAX_PRIORITIES - 1, &xLoRaCommunicateHandle);

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
  const TickType_t timeout = pdMS_TO_TICKS(1);
  for (;;) {
    // Wait for write flag to be ready, clear flag on exit
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, 0x01, pdTRUE, pdFALSE, timeout);
    if ((uxBits & 0x01) == 0x01) {
      _Bool success = mem.readPage(&mem, outBuff); // Flush data to output buffer
    }
  }
}

/* =====================================================================
 *                             LORA HANDLING
 * ===================================================================== */

void vLoRaCommunicate(void *argument) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 2Hz
    for (;;) {
        char payload[16] = {
            LORA_HEADER_AVD1, 
            accel[0], accel[1], accel[2],
            accel[3], accel[4], accel[5],
            magnet[0], magnet[1],
            ':', ')'
        };
        Load_And_Send_To_LoRa(payload, &LoRa_Regs);
        // Block until transmit enable task group bit is set
        //  this is managed within an ISR triggered by the LoRa module
        // ...
        // Repeat 2 more times to delivery all payloads
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* =====================================================================
 *                            STATE MANAGEMENT
 * ===================================================================== */

void vStateUpdate(void *argument) {
  int ms = 0;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz

  float prevAccel = 0;

  for (;;) {
    // Block until 20ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Increment flight timer period post-launch
    if (currentState != PRELAUNCH)
      ms += 20; // 

    switch (currentState) {
    case PRELAUNCH:
      // if (periods >= 100 || isAccelerationAbove5Gs()) { // Using periods to simulate >2s timing
			CAN_TX(1, 8, 0x0, 0x0, 0x602);
      if (isAccelerationAbove5Gs()) {
				// load and send to lora 
				
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
      if (isAltitudeDropping && isTiltAngleAbove90() && isNegativeVelocity()) {
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
    prevAccel = accelZ;
  }
}

/* =====================================================================
 *                  HIGH RESOLUTION DATA ACQUISITION
 * ===================================================================== */

void vDataAcquisitionH(void *argument) {
  float dt = 0.002;
  float roll, pitch, yaw;

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500Hz
  const TickType_t blockTime  = pdMS_TO_TICKS(0); // Don't need to block
  for (;;) {
    // Block until 2ms interval
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Read and process accelerometer
    accel[0] = read_ACCELL_1(0x09); // Accel X high
    accel[1] = read_ACCELL_1(0x08); // Accel X low
    accel[2] = read_ACCELL_1(0x0B); // Accel Y high
    accel[3] = read_ACCELL_1(0x0A); // Accel Y low
    accel[4] = read_ACCELL_1(0x0D); // Accel Z high
    accel[5] = read_ACCELL_1(0x0C); // Accel Z low
    // Need to adjust this to actual axis
    accelZ = ACCEL_SENSITIVITY_32G * (int16_t)(((uint16_t)accel[4] << 8) | accel[5]);

    // Read and process gyroscope
    gyro[0] = read_GYRO(0x29); // Gyro X high
    gyro[1] = read_GYRO(0x28); // Gyro X low
    gyro[2] = read_GYRO(0x2B); // Gyro Y high
    gyro[3] = read_GYRO(0x2A); // Gyro Y low
    gyro[4] = read_GYRO(0x2D); // Gyro Z high
    gyro[5] = read_GYRO(0x2C); // Gyro Z low
    // Need to adjust this to actual axis
    roll  = GYRO_SENSITIVITY * (int16_t)(((uint16_t)gyro[0] << 8) | gyro[1]);
    pitch = GYRO_SENSITIVITY * (int16_t)(((uint16_t)gyro[2] << 8) | gyro[3]);
    yaw   = GYRO_SENSITIVITY * (int16_t)(((uint16_t)gyro[4] << 8) | gyro[5]);

    // Fuck this device
    magnet[0] = read_MAG(0x29); // Mag X high
    magnet[1] = read_MAG(0x28); // Mag X low
    magnet[2] = read_MAG(0x2B); // Mag Y high
    magnet[3] = read_MAG(0x2A); // Mag Y low
    magnet[4] = read_MAG(0x2D); // Mag Z high
    magnet[5] = read_MAG(0x2C); // Mag Z low

    // Add sensor data and quaternion to dataframe
    mem.append(&mem, HEADER_HIGHRES);
    mem.append(&mem, accel[0]);  // Accel X high byte
    mem.append(&mem, accel[1]);  // Accel X low byte
    mem.append(&mem, accel[2]);  // Accel Y high byte
    mem.append(&mem, accel[3]);  // Accel Y low byte
    mem.append(&mem, accel[4]);  // Accel Z high byte
    mem.append(&mem, accel[5]);  // Accel Z low byte
    mem.append(&mem, gyro[0]);   // Gyro X high byte
    mem.append(&mem, gyro[1]);   // Gyro X low byte
    mem.append(&mem, gyro[2]);   // Gyro Y high byte
    mem.append(&mem, gyro[3]);   // Gyro Y low byte
    mem.append(&mem, gyro[4]);   // Gyro Z high byte
    mem.append(&mem, gyro[5]);   // Gyro Z low byte
    mem.append(&mem, magnet[0]); // Magnet X high byte
    mem.append(&mem, magnet[1]); // Magnet X low byte
    mem.append(&mem, magnet[2]); // Magnet Y high byte
    mem.append(&mem, magnet[3]); // Magnet Y low byte
    mem.append(&mem, magnet[4]); // Magnet Z high byte
    mem.append(&mem, magnet[5]); // Magnet Z low byte

    // Only run calculations when enabled
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, 0x02, pdFALSE, pdFALSE, blockTime);
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
  }
}

/* =====================================================================
 *                    LOW RESOLUTION DATA ACQUISITION
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

    // Read baro data
    uint8_t hTemp = read_BARO(0x1F);
    uint8_t lTemp = read_BARO(0x1E);
    uint8_t xTemp = read_BARO(0x1D);
    float temp    = BARO_TEMP_SENSITIVITY * (((int32_t)hTemp << 16) | ((int32_t)lTemp << 8) | xTemp);

    uint8_t hPress = read_BARO(0x22);
    uint8_t lPress = read_BARO(0x21);
    uint8_t xPress = read_BARO(0x20);
    float press    = BARO_PRESS_SENSITIVITY * (((int32_t)hPress << 16) | ((int32_t)lPress << 8) | xPress);

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
    EventBits_t uxBits = xEventGroupWaitBits(xTaskEnableGroup, 0x04, pdFALSE, pdFALSE, blockTime);
    if ((uxBits & 0x04) == 0x04) {
      // Calculate state
      z.pData[0] = altitude;
      z.pData[1] = (cosine * 9.81 * accelZ - 9.81); // Acceleration measured in m/s^2
      kf.update(&kf, &z);

      union {
        float f;
        uint8_t a[4];
      } u;
      u.f = kf.x.pData[1];
    }
  }
}

// Unsure of actual fix for linker error
// temporary (lol) solution
void _init() {}
