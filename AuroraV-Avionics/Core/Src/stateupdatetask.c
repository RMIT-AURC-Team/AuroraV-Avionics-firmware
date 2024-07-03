#include "tasks.h"
#include "stm32f4xx.h"
#include "gpioControl.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h> 

bool isAccelerationAbove5Gs(void);
void sendVelocityAndAltitude(void);
bool isVelocityDecreasing(void);
bool isAltitudeDropping(void);
bool isTiltAngleAbove90(void);
bool isNegativeVelocity(void);
bool isAltitude1300ft(void);


// Define the states using an enum
enum State {
    PRELAUNCH,
    LAUNCH,
    MOTOR_BURNOUT,
    APOGEE,
    DESCENT
};

// State Update and Event Logging Task Function
void stateupdatetask(void const *argument) {
    enum State currentState = PRELAUNCH;

    for(;;) {

        switch (currentState) {
            case PRELAUNCH:
                // Enable prelaunch state
                // Begin write to flash task
                // Check if acceleration is above 5Gs
                if (isAccelerationAbove5Gs()) {
                    currentState = LAUNCH;
                    // Enable launch state
                    // Add launch event dataframe to buffer
                }
                break;

            case LAUNCH:
                // Send velocity and altitude to aerobrakes via CAN
	              sendVelocityAndAltitude();
						
                // Check if velocity is decreasing
                if (isVelocityDecreasing()) {
                    currentState = MOTOR_BURNOUT;
                    // Enable motor burnout state
                    // Add motor burnout event dataframe to buffer
                }
                break;

            case MOTOR_BURNOUT:
                // Check conditions for apogee state
                if (isAltitudeDropping() && isTiltAngleAbove90() && isNegativeVelocity()) {
                    currentState = APOGEE;
                    // Enable apogee state
                    // Add apogee event dataframe to buffer
                    // Send transmission to trigger apogee E-matches
                }
                break;

            case APOGEE:
                // Check if altitude is 1300ft
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
				
				// Execute at 500Hz interval, but with lower priority than high and low res tasks
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }
}


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