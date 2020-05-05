#include "MotorController.h"
#include "esp32-hal-timer.h"  // timer functions, types, etc
#include "RobotArm.h"  // motor indices, etc
#include "RobotArmStepper.h"  // set frequency function

#include "RobotArmDebug.h"


MotorController motorController;

/*
Updates the frequency of motors and increments timeslice
*/
void IRAM_ATTR motorControllerIsr() {
    portENTER_CRITICAL_ISR(&motorController.timerMux);
    bool idle = true;
    if (baseMotor.currentTimeSlice < baseMotor.timeSlices) {
        idle = false;
        setFrequency(BASE_MOTOR_ID, baseMotor.freqs[baseMotor.currentTimeSlice]);
        baseMotor.currentTimeSlice++;
    }
    if (armAMotor.currentTimeSlice < armAMotor.timeSlices) {
        idle = false;
        // R_DPRINTLN(armAMotor.freqs[armAMotor.currentTimeSlice]);
        setFrequency(ARM_A_MOTOR_ID, armAMotor.freqs[armAMotor.currentTimeSlice]);
        armAMotor.currentTimeSlice++;
    }
    if (armBMotor.currentTimeSlice < armBMotor.timeSlices) {
        idle = false;
        setFrequency(ARM_B_MOTOR_ID, armBMotor.freqs[armBMotor.currentTimeSlice]);
        armBMotor.currentTimeSlice++;
    }
    if (idle) {
        // R_DPRINTLN("MC Idle " + String(baseMotor.freqs[baseMotor.currentTimeSlice-1]) + " " + String(armAMotor.freqs[armAMotor.currentTimeSlice-1]) + " " + String(armBMotor.freqs[armBMotor.currentTimeSlice-1]));
        R_DPRINTLN("MC Idle counters " + String(baseMotor.counter) + " " + String(armAMotor.counter) + " " + String(armBMotor.counter));
        R_DPRINTLN("MC Idle targets " + String(baseMotor.target) + " " + String(armAMotor.target) + " " + String(armBMotor.target));
        timerAlarmDisable(motorController.timer);
    }
    portEXIT_CRITICAL_ISR(&motorController.timerMux);
}


void MotorController::init() {
    timer = timerBegin(MOTOR_CONTROLLER_TIMER_NUMBER, MOTOR_CONTROLLER_PRESCALE, true);
    timerAlarmWrite(timer, MOTOR_CONTROLLER_CMV, true);
    timerAttachInterrupt(timer, &motorControllerIsr, true);
}


void MotorController::enable() {
    timerAlarmEnable(timer);
}