#include "RobotArm.h"
#include "esp32-hal.h"
#include "PickerController.h"
#include "RobotArmDebug.h"  // debug printing
#include "RobotArmCalc.h"
#include "RobotArmCheckpoint.h"

RobotArm robotArm;

// Public functions
void RobotArm::init() {
    // base motor
    MotorConfig baseMotorConfig;
    baseMotorConfig.id = BASE_MOTOR_ID;
    baseMotorConfig.stepPin = BASE_STEP_PIN; baseMotorConfig.dirPin = BASE_DIR_PIN;
    baseMotorConfig.gbRatio = BASE_GB_RATIO;
    baseMotorConfig.maxTime = BASE_MAX_TIME; baseMotorConfig.minTime = BASE_MIN_TIME;
    baseMotorConfig.maxAngle = BASE_MAX_ANGLE; baseMotorConfig.minAngle = BASE_MIN_ANGLE;
    baseMotorConfig.timer = timerBegin(BASE_MOTOR_TIMER_NO, MOTOR_TIMER_PRESCALE, true);
    baseMotorConfig.isr = &basePulseIsr;
    baseMotor.init(baseMotorConfig);

    // arm A motor
    MotorConfig armAMotorConfig;
    armAMotorConfig.id = ARM_A_MOTOR_ID;
    armAMotorConfig.stepPin = ARM_A_STEP_PIN; armAMotorConfig.dirPin = ARM_A_DIR_PIN;
    armAMotorConfig.gbRatio = ARM_A_GB_RATIO;
    armAMotorConfig.maxTime = ARM_A_MAX_TIME; armAMotorConfig.minTime = ARM_A_MIN_TIME;
    armAMotorConfig.maxAngle = ARM_A_MAX_ANGLE; armAMotorConfig.minAngle = ARM_A_MIN_ANGLE;
    armAMotorConfig.timer = timerBegin(ARM_A_MOTOR_TIMER_NO, MOTOR_TIMER_PRESCALE, true);
    armAMotorConfig.isr = &armAPulseIsr;
    armAMotor.init(armAMotorConfig);

    // arm B motor
    MotorConfig armBMotorConfig;
    armBMotorConfig.id = ARM_B_MOTOR_ID;
    armBMotorConfig.stepPin = ARM_B_STEP_PIN; armBMotorConfig.dirPin = ARM_B_DIR_PIN;
    armBMotorConfig.gbRatio = ARM_B_GB_RATIO;
    armBMotorConfig.maxTime = ARM_B_MAX_TIME; armBMotorConfig.minTime = ARM_B_MIN_TIME;
    armBMotorConfig.maxAngle = ARM_B_MAX_ANGLE; armBMotorConfig.minAngle = ARM_B_MIN_ANGLE;
    armBMotorConfig.timer = timerBegin(ARM_B_MOTOR_TIMER_NO, MOTOR_TIMER_PRESCALE, true);
    armBMotorConfig.isr = &armBPulseIsr;
    armBMotor.init(armBMotorConfig);

    // picker
    pickerController.init();
}


/*
Moves the robot arm to the given coordinates. Time for the move is automatically calculated.
*/
void RobotArm::move(float x, float y, float z) {
    R_DPRINTLN("Move called to: " + String(x) + " " + String(y) + " " + String(z));
    // calculate angles of each motor
    float baseAngle, armAAngle, armBAngle, pickerAngle;
    if (coordToAngle(x, y, z, &baseAngle, &armAAngle, &armBAngle, &pickerAngle) != 0) {
        R_DPRINTLN("IK Error");
        return;  // IK error
    }
    R_DPRINTLN("Target angles: " + String(baseAngle) + " " + String(armAAngle) + " " + String(armBAngle) + " " + String(pickerAngle));

    // update internal coordinates
    this->x = x;
    this->y = y;
    this->z = z;

    // calculate time for the move
    int timeMillisBase = baseMotor.calcTimeMillis(baseAngle);
    int timeMillisA = armAMotor.calcTimeMillis(armAAngle);
    int timeMillisB = armBMotor.calcTimeMillis(armBAngle);
    int timeMillis = max(max(timeMillisBase,
                             timeMillisA),
                             timeMillisB);
    timeMillis /= speedMulti;  // apply speed multiplier
    
    // move motors to target angles
    baseMotor.moveTo(baseAngle, timeMillis);
    armAMotor.moveTo(armAAngle, timeMillis);
    armBMotor.moveTo(armBAngle, timeMillis);

    // move picker
    pickerController.move(pickerAngle, timeMillis);
}


/*
Moves the robot arm to the given coordinates in the given time.
*/
void RobotArm::move(float x, float y, float z, int timeMillis) {
    R_DPRINTLN("Move called to: " + String(x) + " " + String(y) + " " + String(z) + " Time: " + String(timeMillis));
    // calculate angles of each motor
    float baseAngle, armAAngle, armBAngle, pickerAngle;
    if (coordToAngle(x, y, z, &baseAngle, &armAAngle, &armBAngle, &pickerAngle) != 0) {
        R_DPRINTLN("IK Error");
        return;  // IK error
    }
    R_DPRINTLN("Target angles: " + String(baseAngle) + " " + String(armAAngle) + " " + String(armBAngle) + " " + String(pickerAngle));

    // update internal coordinates
    this->x = x;
    this->y = y;
    this->z = z;
    
    // move motors to target angles
    baseMotor.moveTo(baseAngle, timeMillis);
    armAMotor.moveTo(armAAngle, timeMillis);
    armBMotor.moveTo(armBAngle, timeMillis);

    // move picker
    pickerController.move(pickerAngle, timeMillis);
}


/*
Set the calibration angle for the given motor.
*/
void RobotArm::setCalibrationAngle(int motorId, float angle) {
    calibrationAngles[motorId] = angle;
}


/*
Sets the current angle for each motor to its predefined calibration angle value.
*/
void RobotArm::calibrate() {
    // set the angle and steps of each motor to a predefined value
    baseMotor.steps = angleToSteps(calibrationAngles[BASE_MOTOR_ID], BASE_GB_RATIO);
    baseMotor.actualAngle = stepsToAngle(baseMotor.steps, BASE_GB_RATIO);
    baseMotor.desiredAngle = baseMotor.actualAngle;
    R_DPRINTLN("Base motor: " + String(baseMotor.steps) + " Angle: " + String(baseMotor.actualAngle));
    
    armAMotor.steps = angleToSteps(calibrationAngles[ARM_A_MOTOR_ID], ARM_A_GB_RATIO);
    armAMotor.actualAngle = stepsToAngle(armAMotor.steps, ARM_A_GB_RATIO);
    armAMotor.desiredAngle = armAMotor.actualAngle;
    R_DPRINTLN("Arm A motor: " + String(armAMotor.steps) + " Angle: " + String(armAMotor.actualAngle));

    armBMotor.steps = angleToSteps(calibrationAngles[ARM_B_MOTOR_ID], ARM_B_GB_RATIO);
    armBMotor.actualAngle = stepsToAngle(armBMotor.steps, ARM_B_GB_RATIO);
    armBMotor.desiredAngle = armBMotor.actualAngle;
    R_DPRINTLN("Arm B motor: " + String(armBMotor.steps) + " Angle: " + String(armBMotor.actualAngle));
}


/*
Executes each checkpoint in order once.
*/
void RobotArm::playback() {
    R_DPRINTLN("Playing back all checkpoints");
    for (int i = 0; i < NUM_CP; i++) {
        // TODO add stop button check here
        executeCheckpoint(i);
    }
}


/*
Returns true if any motor is running, false otherwise.
*/
bool RobotArm::running() {
    return baseMotor.running || armAMotor.running || armBMotor.running || !pickerController.done();
}


/*
Executes a checkpoint.
*/
void RobotArm::executeCheckpoint(int cpIdx) {
    int cpType = checkpoints[cpIdx].type;
    switch (cpType) {
        case CP_TYPE_MOVE_TO: 
        {
            move(checkpoints[cpIdx].x, checkpoints[cpIdx].y, checkpoints[cpIdx].z);
            while (running()) delay(0);  // block until move complete
        }
        break;
        case CP_TYPE_DELAY: 
        {
            delay(checkpoints[cpIdx].duration);
        }
        break;
        case CP_TYPE_VALVE: 
        {
            digitalWrite(checkpoints[cpIdx].pin, checkpoints[cpIdx].state);
        }
        break;
        case CP_TYPE_NONE:
        break;
    }
}


/*
Saves a checkpoint at the given index as a positional checkpoint.
*/
void RobotArm::registerPositionCheckpoint(int cpIdx) {
    R_DPRINTLN("Registering position as checkpoint: " + String(cpIdx) + " position: "  + String(x) + ", " + String(y) + ", " + String(z));
    checkpoints[cpIdx].type = CP_TYPE_MOVE_TO;
    checkpoints[cpIdx].x = x;
    checkpoints[cpIdx].y = y;
    checkpoints[cpIdx].z = z;
}


/*
Saves a checkpoint at the given index as a delay checkpoint for the given duration.
*/
void RobotArm::registerDelayCheckpoint(int cpIdx, int duration) {
    R_DPRINTLN("Registering delay as checkpoint: " + String(cpIdx) + " duration: " + String(duration));
    checkpoints[cpIdx].type = CP_TYPE_DELAY;
    checkpoints[cpIdx].duration = duration;
}


/*
Saves a checkpoint at the given index as a valve state checkpoint.
*/
void RobotArm::registerValveCheckpoint(int cpIdx, bool state) {
    R_DPRINTLN("Registering valve as checkpoint: " + String(cpIdx) + " state: " + String(state));
    checkpoints[cpIdx].type = CP_TYPE_VALVE;
    checkpoints[cpIdx].state = state;
}


/*
Clears a checkpoint.
*/
void RobotArm::clearCheckpoint(int cpIdx) {
    R_DPRINTLN("Clearing checkpoint: " + String(cpIdx));
    checkpoints[cpIdx].type = CP_TYPE_NONE;
}


/*
Clear all checkpoints.
*/
void RobotArm::clearAllCheckpoints() {
    R_DPRINTLN("Clearing all checkpoints");
    for (int i = 0; i < NUM_CP; i++) checkpoints[i].type = CP_TYPE_NONE;
}


void RobotArm::switchPicker(bool state) {
    R_DPRINTLN("Switching picker: " + String(state));
    pickerController.toggle(state);
}