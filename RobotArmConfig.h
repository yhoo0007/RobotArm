#ifndef ROBOT_ARM_CONFIG_H
#define ROBOT_ARM_CONFIG_H

// #define RAD_TO_DEG 57.2958  // 180 / pi  // already defined in Arduino.h

// Motors config
#define NUM_MOTORS 3
#define DEGREE_PER_STEP 0.225  // 1.8 / 8 (microstepping) = 0.225

// base motor
#define BASE_STEP_PIN 22
#define BASE_DIR_PIN 21
#define BASE_GB_RATIO 7.143

#define BASE_MOTOR_TIMER_NO 0
#define BASE_MIN_TIME 100
#define BASE_MAX_TIME 3000

// arm A motor
#define ARM_A_STEP_PIN 4
#define ARM_A_DIR_PIN 16
#define ARM_A_GB_RATIO 50

#define ARM_A_MOTOR_TIMER_NO 1
#define ARM_A_MIN_TIME 300
#define ARM_A_MAX_TIME 3000

// arm B motor
#define ARM_B_STEP_PIN 5
#define ARM_B_DIR_PIN 17
#define ARM_B_GB_RATIO 50

#define ARM_B_MOTOR_TIMER_NO 2
#define ARM_B_MIN_TIME 300
#define ARM_B_MAX_TIME 3500

// picker motor
#define PICKER_TX 27  // same for both esp32s
#define PICKER_RX 14  // same for both esp32s
#define PICKER_STEP_PIN 16
#define PICKER_DIR_PIN 17
#define PICKER_GB_RATIO 1


// Arm properties
#define Z_CENTER_TO_ORIGIN 50
#define ARM_A 320
#define ARM_B 320
#define ARM_A2 ARM_A * ARM_A
#define ARM_B2 ARM_B * ARM_B

#define BASE_MAX_ANGLE 225
#define BASE_MIN_ANGLE -45

#define ARM_A_MAX_ANGLE 90
#define ARM_A_MIN_ANGLE 0

#define ARM_B_MAX_ANGLE 180
#define ARM_B_MIN_ANGLE 20

#define PICKER_MAX_ANGLE 360
#define PICKER_MIN_ANGLE 0


#endif  // library guard
