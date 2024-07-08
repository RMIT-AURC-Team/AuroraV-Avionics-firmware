#ifndef _STATE_H
#define _STATE_H

#include "stdbool.h"
#include "math.h"
#include <stdint.h>

enum State {
  PRELAUNCH,
  LAUNCH,
  MOTOR_BURNOUT,
  APOGEE,
  DESCENT
};

bool isAccelerationAbove5Gs(float accelZ);
void sendVelocityAndAltitude(void);
bool isVelocityDecreasing(float velocity, float previousvelocity, uint8_t velocitydecreasecount);
bool isAltitudeDropping(float altitude, float previousaltitude, uint8_t altitudedecreasecount); 
bool isTiltAngleAbove90(float tilt);
bool isNegativeVelocity(float velocity);
bool isAltitude1300ft(float altitude);

#endif
