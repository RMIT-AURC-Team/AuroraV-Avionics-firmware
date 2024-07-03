#include "main.h"

/* Initialise UART */
void initUART() {
  RCC->APB1ENR  |= RCC_APB1ENR_USART3EN;
  RCC->APB1RSTR ^= RCC_APB1RSTR_USART3RST;
  __asm("NOP");
  __asm("NOP");
  RCC->APB1RSTR ^= RCC_APB1RSTR_USART3RST;
  __asm("NOP");
  __asm("NOP");

  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1RSTR ^= RCC_AHB1RSTR_GPIOBRST;
  __asm("NOP");
  __asm("NOP");
  RCC->AHB1RSTR ^= RCC_AHB1RSTR_GPIOBRST;
  __asm("NOP");
  __asm("NOP");

  /* Configure alternate GPIO pin function */
  GPIOB->MODER  &= ~(GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE10_Msk); // Clear bits to configure in MODER
  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL11_Msk | GPIO_AFRH_AFSEL10_Msk); // Clear bits to configure in AHR

  /* Set GPIO pins to alternate function */
  GPIOB->MODER |= (0x02 << GPIO_MODER_MODE11_Pos)
                | (0x02 << GPIO_MODER_MODE10_Pos);

  /* Enable alternate function of pins */
  GPIOB->AFR[1] |= (0x07 << GPIO_AFRH_AFSEL11_Pos)
                 | (0x07 << GPIO_AFRH_AFSEL10_Pos);

  /* Clear UART registers */
  USART3->BRR &= 0xFFFF0000;                         // Clear lower bits of BRR
  USART3->CR1 &= ~(USART_CR1_M);                     // Set number of bits per transfer to 8
  USART3->CR1 &= ~(USART_CR1_PCE);                   // Disable parity
  USART3->CR1 &= ~(USART_CR1_OVER8);                 // Set to 16 bit over sampling
  USART3->CR2 &= ~(USART_CR2_STOP_Msk);              // Clear bits to configure stop

  USART3->CR2 &= ~(USART_CR3_CTSE | USART_CR3_RTSE); // Disable hardware flow control
  USART3->CR2 &= ~(USART_CR2_CLKEN | USART_CR2_CPHA  // Select asynch mode
                   | USART_CR2_CPOL);

  /* Set required configuration bits */
  USART3->CR2 |= (0x00 << USART_CR2_STOP_Pos);         // Set number of stop bits
  USART3->BRR |= (0x111 << USART_BRR_DIV_Mantissa_Pos) // Mantissa = 22d
               | (0x07 << USART_BRR_DIV_Fraction_Pos); // Fraction = 13d

  /* Enable UART */
  USART3->CR1 |= (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);
}

void transmitByte(uint8_t c) {
  while ((USART3->SR & USART_SR_TXE) == 0);
  USART3->DR = c;
  while ((USART3->SR & USART_SR_TC) == 0);
}

void transmitBytes(uint8_t *c, int count) {
  for (int i = 0; i < count; i++) {
    transmitByte(c[i]);
  }
}

// Task Handles
TaskHandle_t xDataAqcquisitionHHandle = NULL;
TaskHandle_t xUARTDebugHandle         = NULL;

// Unsure of actual fix for linker error
// temporary (lol) solution
void _init() {}

Quaternion qRot;                // Global attitude quaternion
float vAttitude[3] = {0, 0, 1}; // Attitude vector
float zUnit[3]     = {0, 0, 1}; // Z unit vector
float cosine       = 0;         // Tilt angle cosine
float tilt         = 0;         // Tilt angle

#define BUFF_SIZE 32000
#define PAGE_SIZE 256
MemBuff mem;
uint8_t buff[BUFF_SIZE];

int main(void) {
  //  configure_RCC_APB1();
  //  configure_RCC_APB2();
  //  configure_RCC_AHB1();

  SystemInit();
  initUART();

  MemBuff_init(&mem, buff, BUFF_SIZE, PAGE_SIZE);
  Quaternion_init(&qRot);

  // Create task handles
  xTaskCreate(vDataAcquisitionH, "DataHighRes", 128, NULL, configMAX_PRIORITIES - 1, &xDataAqcquisitionHHandle);
  // xTaskCreate(vUARTDebug, "DebugComms", 128, NULL, 0, &xUARTDebugHandle);

  vTaskStartScheduler();

  // Should never reach here
  while (1) {
  }
}

// High-Resolution Task Function
void vDataAcquisitionH(void *argument) {

  unsigned int index = 0;
  float dt           = 0.002;

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(2);

  for (;;) {

    /* ========================================
     *             Read sensor data
     * ======================================== */

    float roll;
    uint8_t r[] = {gyroX[index + 0], gyroX[index + 1], gyroX[index + 2], gyroX[index + 3]};
    memcpy(&roll, &r, sizeof(roll));

    float pitch;
    uint8_t p[] = {gyroY[index + 0], gyroY[index + 1], gyroY[index + 2], gyroY[index + 3]};
    memcpy(&pitch, &p, sizeof(pitch));

    float yaw;
    uint8_t y[] = {gyroZ[index + 0], gyroZ[index + 1], gyroZ[index + 2], gyroZ[index + 3]};
    memcpy(&yaw, &y, sizeof(yaw));

    /* ========================================
     *            Append to dataframe
     * ======================================== */

    // Add sensor data and quaternion to dataframe
    mem.append(&mem, HEADER_HIGHRES);
    // TODO: Add sync
    //
    // mem.append(&mem, accel[0]); // Accel X high byte
    // mem.append(&mem, accel[1]); // Accel X low byte
    // mem.append(&mem, accel[2]); // Accel Y high byte
    // mem.append(&mem, accel[3]); // Accel Y low byte
    // mem.append(&mem, accel[4]); // Accel Z high byte
    // mem.append(&mem, accel[5]); // Accel Z low byte
    //
    // mem.append(&mem, gyro[0]); // Gyro X high byte
    // mem.append(&mem, gyro[1]); // Gyro X low byte
    // mem.append(&mem, gyro[2]); // Gyro Y high byte
    // mem.append(&mem, gyro[3]); // Gyro Y low byte
    // mem.append(&mem, gyro[4]); // Gyro Z high byte
    // mem.append(&mem, gyro[5]); // Gyro Z low byte

    /* ========================================
     *            Calculate attitude
     * ======================================== */

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

    index += 4;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// High-Resolution Task Function
void vUARTDebug(void *argument) {

  float dt = 0.002;

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(dt * 1000);

  union {
    float fData;
    uint8_t bData[sizeof(float)];
  } u;

  for (;;) {
    u.fData = qRot.w;
    transmitBytes(u.bData, sizeof(float));
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void vApplicationIdleHook(void) {
}
