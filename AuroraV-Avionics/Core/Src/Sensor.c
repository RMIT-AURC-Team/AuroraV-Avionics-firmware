#include "Sensor.h"

// #inlcude "SPIlib.h"

/***** Senor init and read functions *****/

/**************************************
 * A3G4250D Gyroscope
 **************************************/

void initGyro() {
/**************************************
 * A3G4250D Gyro Initialization
 * Configuration:
 *  - CTRL_REG1: 0xC7 
 *      - ODR (Output Data Rate) = 800Hz
 *      - BW (Bandwidth) = 
 *			- POwer Mode = Normal mode
 *      - All axes (X, Y, Z) enabled
 *************************************/
		
    // spiWrite(G_CTRL_REG1, 0xCF); 
}


void readGyro(int16_t *data) {
    readGyro(data, data + 1, data + 2);
}

void readGyro(int16_t *x, int16_t *y, int16_t *z) {
/**************************************
 Gyro registers:
 *Not using Temp
 OUT_TEMP (26)
 OUT_X_L (28h), OUT_X_H (29h)
 OUT_Y_L (2Ah), OUT_Y_H (2Bh) 
 OUT_Z_L (2Ch), OUT_Z_H (2Dh)
 *************************************/
		// Offsets 
		int g_offx = 0;
		int g_offy = 0;
		int g_offz = 0;
	
		// Buffer 
		uint8_t buff[G_DATA_LENGTH];

    // Read X, Y, Z data starting from OUT_X_L register
    // spiRead(G_OUT_X_L | 0x80, buffer, G_DATA_LENGTH); 

    // Combine high and low bytes, convert to two's complement
    *x = (int16_t)((buff[1] << 8) | buff[0]) + g_offx;
    *y = (int16_t)((buff[3] << 8) | buff[2]) + g_offy;
    *z = (int16_t)((buff[5] << 8) | buff[4]) + g_offz;
}
                     
/**************************************
 * KX134-1211 Accelerometer
 **************************************/

void initAccel() {
/*********************************************************
 * KX134-1211 Accelerometer Initialization
 * No use of Interupts 
 * Configuration:
 *  - CNTL1: 0xD0 
 *      - PC1: 1 (Accelerometer operating)
 *      - RES: 1 (High-performance mode)
 *      - DRDYE: 0 (Data Ready interrupt disabled)
 *      - GSEL<1:0>: 10 (±32g range)
 *  - ODCNTL: 0x2A
 *      - IIR_BYPASS: 1 (IIR filter disabled)
 *      - LPRO: 0 (Low pass filter roll-off at ODR/9)
 *      - FSTUP: 1 (Fast start-up enabled)
 *      - OSA<3:0>: 1010 (Output data rate 800Hz)
 *********************************************************/
	
    // Power down the accelerometer
    //spiWrite(ACCEL_CS_PIN, CNTL1, 0x00); 

    // Set CNTL1 and ODCNTL
    spiWrite(ACCEL_CS_PIN, CNTL1, 0xD0);
    spiWrite(ACCEL_CS_PIN, ODCNTL, 0xAA);
}


void readAccel(int16_t *data) {
    readAccel(data, data + 1, data + 2); 
}

void readAccel(int16_t *x, int16_t *y, int16_t *z) {
/*********************************************************
 * Accelerometer registers:
 * - XOUT_L (0x08), XOUT_H (0x09)
 * - YOUT_L (0x0A), YOUT_H (0x0B)
 * - ZOUT_L (0x0C), ZOUT_H (0x0D) 
 *********************************************************/	
	
    uint8_t buff[A_TO_READ]; // Buffer to store 6 bytes of data (x, y, z)

    // Start reading from XOUT_L register (0x08) with the read bit set (0x80)
    // spiRead(ACCEL_CS_PIN, XOUT_L | 0x80, 6, buff); 

    // Combine high and low bytes for each axis, handling two's complement
    *x = (int16_t)((buff[1] << 8) | buff[0]); // X-axis
    *y = (int16_t)((buff[3] << 8) | buff[2]); // Y-axis
    *z = (int16_t)((buff[5] << 8) | buff[4]); // Z-axis
}













/******** Magnetometer ********/

void initMagnet(){
  uint8_t oversampling = QMC5883L_CONFIG_OS512;
  uint8_t range = QMC5883L_CONFIG_2GAUSS;
  uint8_t rate = QMC5883L_CONFIG_200HZ;
  uint8_t mode = QMC5883L_CONFIG_CONT;

  // Reset device and set configuration
  i2cWrite(MAGNET,QMC5883L_RESET,0x01);
  i2cWrite(MAGNET,QMC5883L_CONFIG,oversampling|range|rate|mode);  
}

void readMagnet(int16_t *data){
  readMagnet(data, data+1, data+2);;
}

void readMagnet(int16_t *x, int16_t *y, int16_t *z){
  uint8_t buff[M_TO_READ];
  int regAddress = QMC5883L_X_LSB;
  i2cRead(MAGNET, regAddress, M_TO_READ, buff); // read the gyro data from the ITG3200
  *x = (buff[1] << 8) | buff[0];
  *y = (buff[3] << 8) | buff[2];
  *z = (buff[5] << 8) | buff[4];
}

/******** Barometer ********/
void initBarometer(){
	i2cWrite(BARO, SPL06_PRS_CFG, SPL06_PRS_RATE_128);	// Pressure single sample
	i2cWrite(BARO, SPL06_TMP_CFG, SPL06_TMP_RATE_128
	                            | SPL06_TMP_EXT);	      // Temperature single sample with external temp
	i2cWrite(BARO, SPL06_MEAS_CFG, 0x07);	              // continuous temp and pressure measurement
	i2cWrite(BARO, SPL06_CFG_REG, 0x00);	              // FIFO Pressure measurement  
}

void readBaro(int32_t *data) {
  uint8_t buff[B_TO_READ];
  int regAddress = SPL06_PRS_B2;
  i2cRead(BARO, regAddress, B_TO_READ, buff); 

  // Read in and format pressure data to two's complement
  data[0] = (buff[0] << 8) | buff[1];
  data[0] = data[0] << 8 | buff[2];

  if(data[0] & (1 << 23))
    data[0] = data[0] | 0XFF000000;   

  // Read in and format temperature data to two's complement
  data[1] = (buff[3] << 8) | buff[4];
  data[1] = (data[1] << 8) | buff[5];

  if(data[1] & (1 << 23))
    data[1] = data[1] | 0xFF000000;   
}


