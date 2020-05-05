#include "esp32-hal.h"  // timer functions, pinMode, et#include "esp"
#include "Picker.h"
#include "RobotArmConfig.h"
#include "RobotArmCalc.h"

#define PICKER_MOTOR_ID 0
#define PICKER_ENA_PIN 4


hw_timer_t* motorControllerTimer = NULL;
portMUX_TYPE motorControllerTimerMux = portMUX_INITIALIZER_UNLOCKED;

float angle = 0.0;
int timeMillis;


void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, PICKER_RX, PICKER_TX);
    Serial.println("Setting up motor");
    picker.init();
    pinMode(PICKER_ENA_PIN, OUTPUT);
    Serial.println("Setup complete");
}


void loop() {
    serveSerial();
}


void serveSerial() {
    while (Serial1.available()) {
        char chr;
        String buf = "";
        do {
            chr = Serial1.read();
            buf += String(chr);
        } while (Serial1.available() && chr != '\n');

        String paramType = buf.substring(0, 1);

        if (paramType == "E") {
            Serial.println("Enabling motor");
            digitalWrite(PICKER_ENA_PIN, LOW);
        } else if (paramType == "D") {
            Serial.println("Disabling motor");
            digitalWrite(PICKER_ENA_PIN, HIGH);
        } else if (paramType == "G") {
            Serial.println("Moving motor");
            picker.moveTo(angle, timeMillis);
            while (picker.running) yield();
            Serial1.print('D');
            Serial.println("Done");
        } else if (paramType == "P") {
            angle = buf.substring(1).toFloat();
            Serial.println("Steps: " + String(angle));
        } else if (paramType == "T") {
            timeMillis = buf.substring(1).toInt();
            Serial.println("Time: " + String(timeMillis));
        }
    }
}