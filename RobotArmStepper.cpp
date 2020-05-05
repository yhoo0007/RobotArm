#include <stdint.h>
#include "RobotArmStepper.h"
#include "RobotArmDebug.h"  // debug printing
#include "RobotArm.h"  // motor indices, etc
#include "RobotArmConfig.h"  // motor params
#include "RobotArmCalc.h"  // steps/angle conversions
#include "MotorController.h"  // update frequency of motor controller for speed calculations
#include "math.h"  // abs function
#include "esp32-hal.h"  // timer functions, pinMode, etc


extern MotorController motorController;

void IRAM_ATTR setFrequency(int motorId, uint16_t frequency) {
    if (frequency > 0) {
        int cmv = round((MOTOR_TIMER_FREQ / 2) / (double)frequency);
        switch (motorId) {
        case BASE_MOTOR_ID:
            timerAlarmWrite(baseMotor.timer, cmv, true);
            break;
        case ARM_A_MOTOR_ID:
            timerAlarmWrite(armAMotor.timer, cmv, true);
            break;
        case ARM_B_MOTOR_ID:
            timerAlarmWrite(armBMotor.timer, cmv, true);
            break;
        }
    }
}


// !!! ISRs are tied to timers !!!
void IRAM_ATTR basePulseIsr() {
    portENTER_CRITICAL_ISR(&baseMotor.timerMux);
    if (baseMotor.counter < baseMotor.target) {
        bool state = digitalRead(baseMotor.stepPin);
        digitalWrite(baseMotor.stepPin, !state);
        if (state) baseMotor.counter++;
    } else {
        baseMotor.running = false;
        timerAlarmDisable(baseMotor.timer);
    }
    portEXIT_CRITICAL_ISR(&baseMotor.timerMux);
}

void IRAM_ATTR armAPulseIsr() {
    portENTER_CRITICAL_ISR(&armAMotor.timerMux);
    if (armAMotor.counter < armAMotor.target) {
        bool state = digitalRead(armAMotor.stepPin);
        digitalWrite(armAMotor.stepPin, !state);
        if (state) armAMotor.counter++;
    } else {
        armAMotor.running = false;
        timerAlarmDisable(armAMotor.timer);
    }
    portEXIT_CRITICAL_ISR(&armAMotor.timerMux);
}

void IRAM_ATTR armBPulseIsr() {
    portENTER_CRITICAL_ISR(&armBMotor.timerMux);
    if (armBMotor.counter < armBMotor.target) {
        bool state = digitalRead(armBMotor.stepPin);
        digitalWrite(armBMotor.stepPin, !state);
        if (state) armBMotor.counter++;
    } else {
        armBMotor.running = false;
        timerAlarmDisable(armBMotor.timer);
    }
    portEXIT_CRITICAL_ISR(&armBMotor.timerMux);
}


void RobotArmStepper::init(MotorConfig config) {
    R_DPRINTLN("Initializing motor " + String(config.id));
    id = config.id;
    stepPin = config.stepPin;
    dirPin = config.dirPin;
    gbRatio = config.gbRatio;
    maxTime = config.maxTime;
    minTime = config.minTime;
    stepRange = angleToSteps(config.maxAngle - config.minAngle, gbRatio);
    timer = config.timer;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    timerAttachInterrupt(timer, config.isr, true);
    
    // to ensure timer initializes properly
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);
    timerAlarmDisable(timer);
}


void RobotArmStepper::moveSteps(int stepsToMove, int timeMillis) {
    R_DPRINTLN(String(id) + " Stepper move called; Steps: " + String(stepsToMove) + " Time: " + String(timeMillis));
    if (stepsToMove == 0) return;
    steps += stepsToMove;  // update internal steps to actual amount moved
    actualAngle += stepsToAngle(stepsToMove, gbRatio);  // update internal angle to actual amount moved
    digitalWrite(dirPin, stepsToMove < 0);
    R_DPRINTLN("Internal steps: " + String(steps) + " Angle: " + String(actualAngle));

    // calculate speed profile
    stepsToMove = abs(stepsToMove);
    timeSlices = timeMillis / MOTOR_CONTROLLER_UPDATE_INTERVAL_MILLIS;

    int fMax = stepsToMove / (float)timeMillis * 1000 * 2;
    int fDelta = fMax / timeSlices * 2;
    int correctiveSteps = stepsToMove * 100;

    // form frequencies array
    int midPoint = timeSlices / 2;  // 43 / 2 = 21
    freqs = (uint16_t*)malloc(sizeof(uint16_t) * timeSlices);
    for (int i = 0; i < timeSlices; i++) {
        freqs[i] = i < midPoint ?
                   fDelta * i :
                   fDelta * (timeSlices - i);  // ceil to avoid arm slow drifting to target near end of movement
        correctiveSteps -= freqs[i];
    }

    int i = 0;
    while (correctiveSteps > 0) {
        freqs[i]++;
        correctiveSteps--;
        i = (i + 1) % timeSlices;
    }

    // set pulse target and reset variables
    target = stepsToMove;
    counter = 0;
    currentTimeSlice = 0;
    R_DPRINTLN(String(id) + " Target: " + String(target) + " Last F: " + String(freqs[timeSlices-1]));

    running = true;
    motorController.enable();
    timerAlarmEnable(timer);
}


void RobotArmStepper::move(float deltaAngle, int timeMillis) {
    float targetAngle = desiredAngle + deltaAngle;
    moveTo(targetAngle, timeMillis);
}


void RobotArmStepper::moveTo(float targetAngle, int timeMillis) {
    float angleToMove = targetAngle - actualAngle;  // correct angle errors
    int stepsToMove = angleToSteps(angleToMove, gbRatio);
    desiredAngle = targetAngle;
    moveSteps(stepsToMove, timeMillis);
}


int RobotArmStepper::calcTimeMillis(int targetSteps) {
    int steps_ = targetSteps - steps;
    return (maxTime - minTime) * sin(((double)abs(steps_) * M_PI) / (stepRange * 2)) + 100;
}


int RobotArmStepper::calcTimeMillis(float targetAngle) {
    int targetSteps = angleToSteps(targetAngle, gbRatio);
    return calcTimeMillis(targetSteps);
}