#ifndef ROBOT_ARM_STEPPER_H
#define ROBOT_ARM_STEPPER_H

#include <stdint.h>
#include "esp32-hal-timer.h"  // timer functions, types, etc

#define MOTOR_TIMER_PRESCALE 80
#define MOTOR_TIMER_FREQ 1000000


void setFrequency(int motorId, uint16_t frequency);
void basePulseIsr();
void armAPulseIsr();
void armBPulseIsr();

typedef struct {
    int id;
    int stepPin;
    int dirPin;

    float gbRatio;
    int maxTime;
    int minTime;
    float maxAngle;
    float minAngle;

    hw_timer_t* timer;
    // void* isr;
    void (*isr)();
} MotorConfig;

class RobotArmStepper {
    public:
    // void init(int motorId);  // loads motor parameters
    void init(MotorConfig config);
    void move(float deltaAngle, int timeMillis);
    void moveTo(float angle, int timeMillis);
    int calcTimeMillis(int steps);
    int calcTimeMillis(float angle);

    int id = 0;
    float actualAngle = 0;
    float desiredAngle = 0;
    int steps = 0;

    // when 'running' is set to true, robot arm's motor controller will periodically update the 
    // frequency of this motor's channel in the pulse timer. Frequencies are updated 
    bool running = false;
    int currentTimeSlice = 0;
    int timeSlices = 0;
    int counter = 0;  // stores number of pulses sent
    int target = 0;  // stores target steps to move
    uint16_t *freqs = NULL;  // stores frequency at each timeslice
    hw_timer_t* timer = NULL;
    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

    int stepPin;
    int dirPin;
    float gbRatio;  // gearbox or belt reduction ratios
    
    // used to calculate time for movements
    int maxTime;
    int minTime;
    int stepRange;

    private:
    void moveSteps(int stepsToMove, int timeMillis);
};


#endif  // library guard
