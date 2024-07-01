#ifndef SENSOR_H
#define SENSOR_H


/**************************************
 * A3G4250D Gyroscope
 **************************************/
 
//#define GYRO ()  					// Gyro Address
#define G_CTRL_REG1 0x20 		// Control Register 1
#define G_OUT_X_L 0x28   		// Output Register X Low Byte
#define G_TO_READ 6 				// 2 bytes for each axis (X, Y, Z)
#define G_SENSITIVITY 8.75  // in mdps/digit

void initGyro();
void readGyro(int16_t *data);
void readGyro(int16_t *x, int16_t *y, int16_t *z);

/**************************************
 * KX134-1211 Accelerometer
 **************************************/
 
//#define ACCEL ()  				// Accel Address
#define CNTL1 0x18
#define CNTL2 0x19
#define CNTL3 0x1D
#define CNTL4 0x1E
#define CNTL5 0x1F
#define CNTL6 0x20
#define ODCNTL 0x21
#define A_XOUT_L 0x08				// Output Register X Low Byte
  		
#define A_TO_READ 6 				// 2 bytes for each axis (X, Y, Z)
#define A_SENSITIVITY 1024  // We are using +-32g range, 

void initAccel();
void readAccel(int16_t *data);
void readAccel(int16_t *x, int16_t *y, int16_t *z);

/**************************************
 * Magnetometer
 **************************************/

void initMagnet();
void readMagnet(int16_t *data);
void readMagnet(int16_t *x, int16_t *y, int16_t *z);


/**************************************
 * Barometer
 **************************************/
 

void initBarometer();
void readBaro(int32_t *data);
void readPressure(int32_t *pressure);
void readTemp(int32_t *temp);

#endif
