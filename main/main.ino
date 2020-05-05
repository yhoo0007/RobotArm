#include <BlynkSimpleEsp32.h>
#include "RobotArm.h"
#include "MotorController.h"


bool playback = false;
float x, y, z;  // buffer to receive blynk parameters
int cpIdx = 0;


void setup() {
    Serial.begin(115200);
    Serial.println("Setting up robot arm");
    robotArm.init();
    robotArm.setCalibrationAngle(BASE_MOTOR_ID, 90);
    robotArm.setCalibrationAngle(ARM_A_MOTOR_ID, 45);
    robotArm.setCalibrationAngle(ARM_B_MOTOR_ID, 90);
    robotArm.calibrate();
    Serial.println("Setting up motor controller");
    motorController.init();
    Serial.println("Setting up Blynk");
    setupBlynk();
    Serial.println("Setting up CAN bus");
    setupCAN();
    Serial.println("Setup complete\n");
}


void loop() {
    Blynk.run();
    if (playback || startSignal()) {
        Serial.println("Playing back");
        robotArm.playback();  // blocking playback
        Serial.println("Cycle completed");
        endSignal();
    }
}


void setupBlynk() {
    char auth[] = "";
    char ssid[] = "";
    char pass[] = "";
    Blynk.begin(auth, ssid, pass);
    x = 0; y = 450; z = 50;
    Blynk.virtualWrite(V0, x);
    Blynk.virtualWrite(V1, y);
    Blynk.virtualWrite(V2, z);
    Blynk.virtualWrite(V12, robotArm.speedMulti);
}


void setupCAN() {
  
}


bool startSignal() {
    return false;
}


void endSignal() {
    
}


BLYNK_WRITE(V0) {  // X position slider
    x = param.asFloat();
}


BLYNK_WRITE(V1) {  // Y position slider
    y = param.asFloat();
}


BLYNK_WRITE(V2) {  // Z position slider
    z = param.asFloat();
}


BLYNK_WRITE(V3) {  // calibrate position button
    robotArm.calibrate();
}


BLYNK_WRITE(V4) {  // step checkpoints
    cpIdx = param.asInt();
    Blynk.virtualWrite(V5, cpIdx);
    robotArm.executeCheckpoint(cpIdx);
    x = robotArm.checkpoints[cpIdx].x;
    y = robotArm.checkpoints[cpIdx].y;
    z = robotArm.checkpoints[cpIdx].z;
    Blynk.virtualWrite(V0, robotArm.checkpoints[cpIdx].x);
    Blynk.virtualWrite(V1, robotArm.checkpoints[cpIdx].y);
    Blynk.virtualWrite(V2, robotArm.checkpoints[cpIdx].z);
}


BLYNK_WRITE(V5) {  // checkpoint selector
    cpIdx = param.asInt();
}


BLYNK_WRITE(V6) {  // set position checkpoint
    robotArm.registerPositionCheckpoint(cpIdx);
}


BLYNK_WRITE(V7) {  // set delay checkpoint
    robotArm.registerDelayCheckpoint(cpIdx, param.asInt());
}


BLYNK_WRITE(V8) {  // set valve checkpoint
    robotArm.registerValveCheckpoint(cpIdx, param.asInt());
}


BLYNK_WRITE(V9) {  // clear all checkpoints
    robotArm.clearAllCheckpoints();
}


BLYNK_WRITE(V10) {  // playback
    playback = param.asInt();
}


BLYNK_WRITE(V11) {  // picker enable
    robotArm.switchPicker(param.asInt());
}


BLYNK_WRITE(V12) {  // speed multi
    float multi = param.asFloat();
    Serial.println("Speed multiplier set to: " + String(multi));
    robotArm.speedMulti = multi;
}


BLYNK_WRITE(V13) {  // initiate move
    robotArm.move(x, y, z);
    while (robotArm.running()) yield();
    Blynk.virtualWrite(V13, 0);
}