#ifndef PICKER_H
#define PICKER_H

#include <stdint.h>
#include "esp32-hal.h"  // timer functions, pinMode, etc


void setFrequency();
void motorControllerRoutine();
void pulseIsr();

class Picker {
    public:
    void init();
    void move(float deltaAngle, int timeMillis);
    void moveTo(float targetAngle, int timeMillis);
    void moveSteps(int stepsToMove, int timeMillis);

    hw_timer_t* motorControllerTimer = NULL;
    portMUX_TYPE motorControllerTimerMux = portMUX_INITIALIZER_UNLOCKED;

    hw_timer_t* pulseTimer = NULL;
    portMUX_TYPE pulseTimerMux = portMUX_INITIALIZER_UNLOCKED;

    int stepPin;
    int dirPin;
    float gbRatio;

    float actualAngle;
    float desiredAngle;

    int timeSlices;
    uint16_t *freqs;
    int target;
    int counter;
    int currentTimeSlice;
    bool running;
};

extern Picker picker;


#endif