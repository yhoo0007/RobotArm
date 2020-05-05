#include "PickerController.h"
#include "HardwareSerial.h"
#include "RobotArmConfig.h"




void PickerController::init() {
    Serial1.begin(115200, SERIAL_8N1, PICKER_TX, PICKER_RX);
}


void PickerController::move(float angle, int timeMillis) {
    Serial1.print("P"); Serial1.println(angle);
    Serial1.println("T" + String(timeMillis));
    Serial1.println("G");
    running = true;
}


bool PickerController::done() {
    if (Serial1.available() && Serial1.read() == 'D') {
        running = false;
    }
    return !running;
}


void PickerController::toggle(bool state) {
    if (state) Serial1.println("E");
    else Serial1.println("D");
}