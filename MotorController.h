#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "esp32-hal-timer.h"  // timer functions, types, etc


#define MOTOR_CONTROLLER_TIMER_NUMBER 3
#define MOTOR_CONTROLLER_PRESCALE 80
#define MOTOR_CONTROLLER_CMV 10000  // prescale + cmv causes isr to be called at 100Hz, i.e. 10ms between calls
#define MOTOR_CONTROLLER_UPDATE_INTERVAL_MILLIS 10


void motorControllerIsr();
class MotorController {
    public:
    void init();
    void enable();

    hw_timer_t* timer;
    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
};

extern MotorController motorController;

#endif
