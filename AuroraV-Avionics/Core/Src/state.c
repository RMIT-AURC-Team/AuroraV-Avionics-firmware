#include "state.h"

// Functions for sensor checks and actions
bool isAccelerationAbove5Gs(float accelZ) {
  // Check if acceleration is above 5Gs
		if (accelZ >= 5) {
		return true;
  }
	
  return false;
}

void sendVelocityAndAltitude() {
  // Send velocity and altitude to aerobrakes via CAN
}

bool isVelocityDecreasing(float velocity, float previousvelocity, uint8_t velocitydecreasecount) {

	// read in current Velocity
	float currentvelocity = velocity;

	// Counts the number of times the velocity is negative.  
	if (currentvelocity < previousvelocity)
	{
		velocitydecreasecount++; 
		// set buzzer to check if this works
		void buzzer(void);
	}
	else if (currentvelocity > previousvelocity)
	{
		velocitydecreasecount = 0; 
	}
	else if (velocitydecreasecount == 5)
	{
		return true; 
	}

	previousvelocity = velocity; 
	
  return false;
}

bool isAltitudeDropping(float altitude, float previousaltitude, uint8_t altitudedecreasecount) {

	// read in current Alitiude
	float currentaltitude = altitude;

	// Counts the number of times the velocity is negative.  
	if (currentaltitude < previousaltitude)
	{
		altitudedecreasecount++; 
		// set buzzer to check if this works
		void buzzer(void);
	}
	else if (currentaltitude > previousaltitude)
	{
		altitudedecreasecount = 0; 
	}
	else if (altitudedecreasecount == 5)
	{
		return true; 
	}

	previousaltitude = altitude; 
	
  return false;	

}

bool isTiltAngleAbove90(float tilt) {
  // Check if tilt angle is above 90 degrees
		if (tilt > 90) {
		return true;
  }
	
  return false;
}

bool isNegativeVelocity(float velocity) {
  // Check if velocity is negative
	
	if (velocity < 0) {
		return true;
  }

  return false;
}

bool isAltitude1300ft(float altitude) {
  // Check if altitude is 1300ft
	if (altitude >= 396.24) {
		return true;
  }
	
  return false;
}
