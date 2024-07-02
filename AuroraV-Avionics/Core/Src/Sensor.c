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
		
    spiWrite(GYRO, G_CTRL_REG1, 0xCF); 
}


void readGyro(int16_t *data) {
    readGyro(data, data + 1, data + 2);
}

void readGyro(int16_t *x, int16_t *y, int16_t *z) {
/**************************************
 Gyro registers:
 *Not using Temp
 OUT_TEMP (26)
 - OUT_X_L (28h), OUT_X_H (29h)
 - OUT_Y_L (2Ah), OUT_Y_H (2Bh) 
 - OUT_Z_L (2Ch), OUT_Z_H (2Dh)
 *************************************/
		// Offsets 
		int g_offx = 0;
		int g_offy = 0;
		int g_offz = 0;
	
		// Buffer 
		uint8_t buff[G_TO_READ];

    // Read X, Y, Z data starting from OUT_X_L register
    spiRead(G_OUT_X_L | 0x80, buffer, G_TO_READ); 

    // Combine high and low bytes, convert to two's complement
    *x = (int16_t)((buff[1] << 8) | buff[0]) + g_offx;
    *y = (int16_t)((buff[3] << 8) | buff[2]) + g_offy;
    *z = (int16_t)((buff[5] << 8) | buff[4]) + g_offz;
}

void calcGyro(int16_t *data) {

    // Convert raw values to degrees per second (dps)
    for (int i = 0; i < 3; i++) {
        data[i] = (int16_t)((data[i] * G_SENSITIVITY) / 1000.0); 
    }
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
    spiWrite(ACCEL, A_CNTL1, 0xD0);
    spiWrite(ACCEL, A_ODCNTL, 0xAA);
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
    // spiRead(ACCEL, XOUT_L | 0x80, A_TO_READ, buff); 

    // Combine high and low bytes for each axis, handling two's complement
    *x = (int16_t)((buff[1] << 8) | buff[0]); // X-axis
    *y = (int16_t)((buff[3] << 8) | buff[2]); // Y-axis
    *z = (int16_t)((buff[5] << 8) | buff[4]); // Z-axis
}

void calcAccel(int16_t* rawData, float* accelG) {
 
    // Convert raw data to acceleration in g's
    for (int i = 0; i < 3; i++) {
        accelG[i] = (float)rawData[i] / A_SENSITIVITY;
    }
}


/**************************************
 *  LIS3MDL Magnetometer
 **************************************/

void initMagnet() {
/*********************************************************
 * LIS3MDL Magnetometer Initialization
 * No use of Interrupts 
 * Configuration:
 *  - CTRL_REG1: 0x7E
 *      - TEMP_EN: 1 (Temperature sensor enabled)
 *      - OM1, OM0: 11 (Ultra-high-performance mode for X and Y)
 *      - DO2, DO1, DO0: 111 (80 Hz output data rate)
 *      - FAST_ODR: 1 (Fast output data rate enabled)
 *      - ST: 1 (Self-test enabled)
 *  - CTRL_REG2: 0x60
 *      - FS1, FS0: 11 (+/- 16 gauss full scale)
 *  - CTRL_REG4: 0x0C
 *      - OMZ1, OMZ0: 11 (Ultra-high-performance mode for Z)
 *********************************************************/    

    // Write configuration 
    spiWrite(MAGNET, M_CTRL_REG1, 0x7E);  // Enable temperature sensor, ultra-high-performance, 80 Hz ODR
    spiWrite(MAGNET, M_CTRL_REG2, 0x60);  // +/- 16 gauss full scale
    spiWrite(MAGNET, M_CTRL_REG4, 0x0C);  // Ultra-high-performance
}

void readMagnet(int16_t *data) {
    readMagnet(data, data + 1, data + 2);
}

void readMagnet(int16_t *x, int16_t *y, int16_t *z) {
/*********************************************************
 * Magnetometer Registers:
 *  - OUT_X_L (0x28), OUT_X_H (0x29)
 *  - OUT_Y_L (0x2A), OUT_Y_H (0x2B)
 *  - OUT_Z_L (0x2C), OUT_Z_H (0x2D)
 *********************************************************/
	
    uint8_t buff[M_TO_READ]; 

    // SPI read 
    spiRead(MAGNET, M_OUT_X_L | 0x80, M_TO_READ, buff); 

    // Combine high and low bytes and convert to two's complement
    *x = (int16_t)((buff[1] << 8) | buff[0]);
    *y = (int16_t)((buff[3] << 8) | buff[2]);
    *z = (int16_t)((buff[5] << 8) | buff[4]);
}

void calcMagnet(int16_t *rawData, float *magnetGauss) {
	
    // Convert raw data to magnetic field strength in gauss
    for (int i = 0; i < 3; i++) {
        magnetGauss[i] = (float)rawData[i] / M_SENSITIVITY;
    }
}

/**************************************
 * BMP581 Barometer
 **************************************/

void initBaro() {
/**********************************************************************
 * BMP581 Initialization
 * No use of Interrupts or FIFO
 * Configuration:
 *  - ODR_CONFIG (0x37): 0x8F
 *      - deep_dis: 1 (Deep standby disabled)
 *      - odr: 1111 (50 Hz output data rate)
 *      - pwr_mode: 01 (Normal Mode)
 **********************************************************************/

    spiWrite(BARO, B_ODR_CONFIG_REG, 0x8F); 
	
}

void readBaro(int32_t *data) {
    /*********************************************************
     * BMP581 Registers:
     *  - PRESS_XLSB_REG (0x20)
     *  - PRESS_LSB_REG (0x21)
     *  - PRESS_MSB_REG (0x22)
     *  - TEMP_XLSB_REG (0x1D)
     *  - TEMP_LSB_REG (0x1E)
     *  - TEMP_MSB_REG (0x1F)
     *********************************************************/

    uint8_t temp_buff[3]; 
    uint8_t press_buff[3];

    // Read pressure data starting from PRESS_XLSB_REG (0x20)
    spiRead(BARO, B_PRESS_XLSB_REG | 0x80, press_buff, 3); 

    // Read temperature data starting from TEMP_XLSB_REG (0x1D)
    spiRead(BARO, B_TEMP_XLSB_REG | 0x80, temp_buff, 3);

    // Combine pressure data into a 24-bit raw value
    data[0] = (int32_t)((press_buff[0] << 16) | (press_buff[1] << 8) | press_buff[2]); 

    // Combine temperature data into a 24-bit raw value
    data[1] = (int32_t)((temp_buff[0] << 16) | (temp_buff[1] << 8) | temp_buff[2]);  
}

// Function to calculate temperature (in °C) from raw temperature data
void calcTemp(int32_t raw_temp, float *temp) {
    *temp = (float)raw_temp / B_TEMP_SCALE; // Convert raw temperature to °C
}

// Function to calculate pressure (in Pa) from raw pressure data
void calcPress(int32_t raw_press, float *pressure) {
    *pressure = (float)raw_press / B_PRESS_SCALE; // Convert raw pressure to Pa
}












