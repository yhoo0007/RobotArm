#include "Picker.h"
#include "RobotArmConfig.h"
#include "RobotArmCalc.h"
#include "RobotArmDebug.h"
#include "esp32-hal.h"  // timer functions, pinMode, etc

#define MOTOR_CONTROLLER_TIMER_NO 0
#define MOTOR_CONTROLLER_PRESCALE 80
#define MOTOR_CONTROLLER_CMV 10000  // prescale + cmv causes isr to be called at 100Hz, i.e. 10ms between calls
#define MOTOR_CONTROLLER_UPDATE_INTERVAL_MILLIS 10

#define PICKER_MOTOR_TIMER_NO 1
#define PICKER_MOTOR_PRESCALE 80
#define PICKER_MOTOR_FREQ 1000000


Picker picker;


void IRAM_ATTR setFrequency(int frequency) {
    if (frequency > 0) {
        int cmv = round((PICKER_MOTOR_FREQ / 2) / (double)frequency);
        timerAlarmWrite(picker.pulseTimer, cmv, true);
    }
}

void IRAM_ATTR motorControllerRoutine() {
    portENTER_CRITICAL_ISR(&picker.motorControllerTimerMux);
    if (picker.currentTimeSlice < picker.timeSlices) {
        setFrequency(picker.freqs[picker.currentTimeSlice]);
        picker.currentTimeSlice++;
    } else {
        // R_DPRINTLN("MC Idle counters " + String(picker.counter));
        // R_DPRINTLN("MC Idle targets " + String(picker.target));
        timerAlarmDisable(picker.motorControllerTimer);
    }
    portEXIT_CRITICAL_ISR(&picker.motorControllerTimerMux);
}


void IRAM_ATTR pulseIsr() {
    portENTER_CRITICAL_ISR(&picker.pulseTimerMux);
    if (picker.counter < picker.target) {
        bool state = digitalRead(picker.stepPin);
        digitalWrite(picker.stepPin, !state);
        if (state) picker.counter++;
    } else {
        picker.running = false;
        timerAlarmDisable(picker.pulseTimer);
    }
    portEXIT_CRITICAL_ISR(&picker.pulseTimerMux);
}


void Picker::init() {
    stepPin = PICKER_STEP_PIN; dirPin = PICKER_DIR_PIN;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    gbRatio = PICKER_GB_RATIO;
    actualAngle = 270 - 90 - 45;  // based on initial arm angle of 45 and 90
    desiredAngle = actualAngle;
    
    // setup motor controller
    motorControllerTimer = timerBegin(MOTOR_CONTROLLER_TIMER_NO, MOTOR_CONTROLLER_PRESCALE, true);
    timerAlarmWrite(motorControllerTimer, MOTOR_CONTROLLER_CMV, true);
    timerAttachInterrupt(motorControllerTimer, &motorControllerRoutine, true);

    // setup pulse timer
    pulseTimer = timerBegin(PICKER_MOTOR_TIMER_NO, PICKER_MOTOR_PRESCALE, true);
    timerAttachInterrupt(pulseTimer, &pulseIsr, true);
    timerAlarmWrite(pulseTimer, 1000, true);
    timerAlarmEnable(pulseTimer);
    timerAlarmDisable(pulseTimer);
}


void Picker::move(float deltaAngle, int timeMillis) {
    float targetAngle = desiredAngle + deltaAngle;
    moveTo(targetAngle, timeMillis);
}


void Picker::moveTo(float targetAngle, int timeMillis) {
    float angleToMove = targetAngle - actualAngle;  // correct angle errors
    int stepsToMove = angleToSteps(angleToMove, gbRatio);
    desiredAngle = targetAngle;
    moveSteps(stepsToMove, timeMillis);
}


void Picker::moveSteps(int stepsToMove, int timeMillis) {
    R_DPRINTLN("Picker move called; Steps: " + String(stepsToMove) + " Time: " + String(timeMillis));
    if (stepsToMove == 0) return;
    actualAngle += stepsToAngle(stepsToMove, gbRatio);  // update internal angle to actual amount moved
    digitalWrite(dirPin, stepsToMove < 0);
    R_DPRINTLN("Internal Angle: " + String(actualAngle));

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
    // R_DPRINTLN("Picker Target: " + String(target) + " Last F: " + String(freqs[timeSlices-1]));

    running = true;
    timerAlarmEnable(motorControllerTimer);
    timerAlarmEnable(pulseTimer);
}
