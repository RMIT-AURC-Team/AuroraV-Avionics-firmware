#ifndef _STATE_H
#define _STATE_H

#include "stdbool.h"

enum State {
  PRELAUNCH,
  LAUNCH,
  MOTOR_BURNOUT,
  APOGEE,
  DESCENT
};

bool isAccelerationAbove5Gs(void);
void sendVelocityAndAltitude(void);
bool isVelocityDecreasing(void);
bool isAltitudeDropping(void);
bool isTiltAngleAbove90(void);
bool isNegativeVelocity(void);
bool isAltitude1300ft(void);

#endif
