#include <BlynkSimpleEsp32.h>
#include "RobotArm.h"
#include "MotorController.h"
#include <ESP32CAN.h>
#include <CAN_config.h>

WidgetBridge bridgeConv(V14);
WidgetBridge bridgeLabel(V16);

bool startSignal = false;
bool playback = false;
float x, y, z;  // buffer to receive blynk parameters
int cpIdx = 0;
int valvePin = 33;
int startLabelCp = -1;

// CAN bus
const gpio_num_t CAN_TX = GPIO_NUM_25;
const gpio_num_t CAN_RX = GPIO_NUM_26;
CAN_device_t CAN_cfg;
#define CONVEYOR_ID 0
#define ROBO_ARM1_ID 1
#define ROBO_ARM2_ID 2
#define FILLING_ID 3
#define CAPPING_ID 4
#define LABEL_ID 5


void setup() {
    Serial.begin(115200);
    Serial.println("Setting up robot arm");
    pinMode(valvePin, OUTPUT);
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
    //setupCAN();
    Serial.println("Setup complete\n");
}


void loop() {
    Blynk.run();
    if (Blynk.connected()) {
      if (playback || startSignal) {
          Serial.println("Playing back");
  //        robotArm.playback();  // blocking playback
          for (int i = 0; i < NUM_CP; i++) {
              if (i == startLabelCp) {
                  startLabel();
              } else {
                  robotArm.executeCheckpoint(i);
              }
          }
          Serial.println("Cycle completed");
          startSignal = false;
          bridgeConv.virtualWrite(V7, ROBO_ARM1_ID);
          Serial.println("Done signal sent!");
      }
    } else {
        Serial.println("BLYNK DISCONNECTED");
        
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
    CAN_cfg.speed = CAN_SPEED_1000KBPS;
    CAN_cfg.tx_pin_id = CAN_TX;
    CAN_cfg.rx_pin_id = CAN_RX;
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
    ESP32Can.CANInit();
}

/*
bool startSignal() {
    CAN_frame_t rx_frame;
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3*portTICK_PERIOD_MS) == pdTRUE) {
      if (rx_frame.MsgID == CONVEYOR_ID && rx_frame.data.u8[0] == 1) {
        return true;  
      }
    }
    return false;
}
*/

/*
void endSignal() {
  
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = ROBO_ARM1_ID;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = 1;
    tx_frame.data.u8[1] = 0;
    tx_frame.data.u8[2] = 0;
    tx_frame.data.u8[3] = 0;
    tx_frame.data.u8[4] = 0;
    tx_frame.data.u8[5] = 0;
    tx_frame.data.u8[6] = 0;
    tx_frame.data.u8[7] = 0;
    ESP32Can.CANWriteFrame(&tx_frame);
    
}
*/

void startLabel() {
  /*
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = ROBO_ARM1_ID;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = 0;
    tx_frame.data.u8[1] = 1;
    tx_frame.data.u8[2] = 0;
    tx_frame.data.u8[3] = 0;
    tx_frame.data.u8[4] = 0;
    tx_frame.data.u8[5] = 0;
    tx_frame.data.u8[6] = 0;
    tx_frame.data.u8[7] = 0;
    ESP32Can.CANWriteFrame(&tx_frame);    
    */
    bridgeLabel.virtualWrite(V10, ROBO_ARM1_ID);
}

BLYNK_CONNECTED() {
    bridgeConv.setAuthToken("");
    bridgeLabel.setAuthToken("");
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


BLYNK_WRITE(V3) {  // initiate label placer
    startLabelCp = cpIdx;
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
    int choice = param.asInt();
    if (choice == 1) {
      Serial.println(String(choice) + " Valve ON");
      choice = 1;
    } else if (choice == 2) {
      Serial.println(String(choice) + " Valve OFF");
      choice = 0;
    } else {
      Serial.println("Unknown choice: " + String(choice));
    }
    robotArm.registerValveCheckpoint(cpIdx, choice, valvePin);
//    robotArm.registerValveCheckpoint(cpIdx, param.asInt());
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

BLYNK_WRITE(V15){
  if(param.asInt() == 1){
    startSignal = true;
  }
}
