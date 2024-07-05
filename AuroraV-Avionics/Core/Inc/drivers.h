#ifndef _DRIVERS_H
#define _DRIVERS_H

#include "stm32f439xx.h"

struct CAN_RX_data {
  unsigned int dataL;   // data high register
  unsigned int dataH;   // data low register
  unsigned int address; // CAN identifer
  uint8_t CAN_number;   // either CAN2 or CAN1
};

struct GPSData {
  char time[15];
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  char latitude[15];
  unsigned long latitude_num;
  char N_S[15];
  char longitude[15];
  unsigned long longitude_num;
  char E_W[15];
  char fix[15];
  char satellites[15];
  char hdop[15];
  char altitude[15];
  uint8_t lock; // 0 = no lock, 1 = lock
};

struct LoRa_Registers {
  uint8_t RegFifo;
  uint8_t RegOpMode;
  uint8_t RegFifoAddrPtr;
  uint8_t RegFifoTxBaseAddr;
  uint8_t RegFifoRxBaseAddr;
  uint8_t RegFifoRxCurrentAddr;
  uint8_t RegIrqFlagsMask;
  uint8_t RegIrqFlags;
  uint8_t RegRxNbBytes;
  uint8_t RegRxHeaderCntValueMsb;
  uint8_t RegRxHeaderCntValueLsb;
  uint8_t RegRxPacketCntValueMsb;
  uint8_t RegRxPacketCntValueLsb;
  uint8_t RegModemStat;
  uint8_t RegPktSnrValue;
  uint8_t RegPktRssiValue;
  uint8_t RegRssiValue;
  uint8_t RegHopChannel;
  uint8_t RegModemConfig1;
  uint8_t RegModemConfig2;
  uint8_t RegSymbTimeoutLsb;
  uint8_t RegPreambleMsb;
  uint8_t RegPreambleLsb;
  uint8_t RegPayloadLength;
  uint8_t RegMaxPayloadLength;
  uint8_t RegHopPeriod;
  uint8_t RegFifoRxByteAddr;
  uint8_t RegFeiMsb;
  uint8_t RegFeiMid;
  uint8_t RegFeiLsb;
  uint8_t RegRssiWideband;
  uint8_t RegDetectOptimize;
  uint8_t RegInvertIQ;
  uint8_t RegDetectionThreshold;
  uint8_t RegSyncWord;
  uint8_t RegInvertQ2;
  uint8_t RegChirpFilter;
  uint8_t RegDioMapping1;
  uint8_t RegDioMapping2;
};

void configure_RCC_APB1(void);
void configure_RCC_APB2(void);
void configure_RCC_AHB1(void);
void configure_MISC_GPIO(void);
void configure_UART3_GPS(void);
void configure_UART6_Serial_2_mini_USB(void);
void configure_SPI1_Sensor_Suite(void);
void configure_SPI3_LoRa(void);
void configure_SPI2_Flash(void);
void configure_external_interrupts(void);
void EXTI1_IRQHandler(void);

void CANGPIO_config(void);
void CAN_Peripheral_config(void);
uint8_t find_empty_CAN_TX_mailbox(uint8_t CAN);
uint8_t CAN_TX(uint8_t CAN, uint8_t datalength, unsigned int dataH, unsigned int dataL, unsigned int address);
uint8_t CAN_RX(struct CAN_RX_data *CAN);

void write_lora_packet(uint8_t address, uint8_t payload);
uint8_t receive_lora_data(uint8_t address);

void write_GYRO(uint8_t address, uint8_t payload);
uint8_t read_GYRO(uint8_t address);
void test_gyro_drivers(void);

uint8_t read_BARO(uint8_t address);
void write_BARO(uint8_t address, uint8_t payload);

void write_MAG(uint8_t address, uint8_t payload);
uint8_t read_MAG(uint8_t address);

void write_ACCELL_1(uint8_t address, uint8_t payload);
uint8_t read_ACCELL_1(uint8_t address);
void write_ACCELL_2(uint8_t address, uint8_t payload);
uint8_t read_ACCELL_2(uint8_t address);

uint8_t read_FLASH(uint8_t address);

void TIM6init(void);
void TIM7init(void);
void buzzer(void);
void send_GPS_messege(char *pointerdata);

void DecodeGPS(char *GPS, struct GPSData *data);
void GPS_test(void);

void Load_And_Send_To_LoRa(char *pointerdata, const struct LoRa_Registers *LoRa_Registers);
void configure_LoRa_module(void);
#endif
