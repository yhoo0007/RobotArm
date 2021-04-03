#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include "RobotArmStepper.h"
#include "PickerController.h"
#include "RobotArmCheckpoint.h"
#include "RobotArmConfig.h"

#define BASE_MOTOR_ID 0
#define ARM_A_MOTOR_ID 1
#define ARM_B_MOTOR_ID 2
#define baseMotor robotArm.motors[BASE_MOTOR_ID]
#define armAMotor robotArm.motors[ARM_A_MOTOR_ID]
#define armBMotor robotArm.motors[ARM_B_MOTOR_ID]


class RobotArm {
    public:
    void init();

    void move(float x, float y, float z);
    void move(float x, float y, float z, int timeMillis);
    
    void setCalibrationAngle(int motorId, float angle);
    void calibrate();

    void playback();
    bool running();
    void executeCheckpoint(int cpIdx);
    void registerPositionCheckpoint(int cpIdx);
    void registerDelayCheckpoint(int cpIdx, int duration);
    void registerValveCheckpoint(int cpIdx, bool state, int pin);
    void clearCheckpoint(int cpIdx);
    void clearAllCheckpoints();

    void switchPicker(bool state);

    RobotArmStepper motors[NUM_MOTORS];
    PickerController pickerController;
    Checkpoint checkpoints[NUM_CP];

    float x, y, z;
    float speedMulti = 1.0;
    float calibrationAngles[NUM_MOTORS];
};

extern RobotArm robotArm;

#endif  // library guard
