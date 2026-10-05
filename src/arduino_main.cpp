#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <uni.h>
#include "controller_callbacks.h"
#include "motors.hpp"

// DRV8833 pins for one motor
const int IN1 = 16;
const int IN2 = 17;
const int DRV_SLEEP = 27;
const int DRV_FAULT = 32;

MotorController robotMotors(IN1, IN2, 18, 19);

void setup() {
    Serial.begin(115200);

    pinMode(DRV_SLEEP, OUTPUT);
    digitalWrite(DRV_SLEEP, HIGH);

    pinMode(DRV_FAULT, INPUT);

    robotMotors.init();

    Serial.println("DRV8833 test start");
    Serial.println("SLEEP enabled");
}

void loop() {
    static unsigned long lastChange = 0;
    static int state = 0;

    delay(10);

    if (millis() - lastChange >= 3000) {
        lastChange = millis();
        state++;

        int fault = digitalRead(DRV_FAULT);
        Serial.print("Fault pin: ");
        Serial.println(fault ? "HIGH" : "LOW");

        switch (state % 3) {
            case 0:
                Serial.println("FORWARD");
                robotMotors.forward(200);
                break;

            case 1:
                Serial.println("BACKWARD");
                robotMotors.backward(200);
                break;

            default:
                Serial.println("STOP");
                robotMotors.stop();
                break;
        }
    }
}