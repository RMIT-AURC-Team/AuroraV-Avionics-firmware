#include "state.h"

// Functions for sensor checks and actions
bool isAccelerationAbove5Gs() {
  // Check if acceleration is above 5Gs
  return false;
}

void sendVelocityAndAltitude() {
  // Send velocity and altitude to aerobrakes via CAN
}

bool isVelocityDecreasing() {
  // Check if velocity is decreasing
  return false;
}

bool isAltitudeDropping() {
  // Check if altitude is dropping
  return false;
}

bool isTiltAngleAbove90() {
  // Check if tilt angle is above 90 degrees
  return false;
}

bool isNegativeVelocity() {
  // Check if velocity is negative
  return false;
}

bool isAltitude1300ft() {
  // Check if altitude is 1300ft
  return false;
}
