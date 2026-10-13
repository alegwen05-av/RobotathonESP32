// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <uni.h>
#include "controller_callbacks.h"
#include "motors.hpp"

// Motor setup using uPesy ESP32 pins
const int IN1 = 16;
const int IN2 = 17;
const int IN3 = 18;
const int IN4 = 19;
MotorController robotMotors(IN1, IN2, IN3, IN4); //makes an object of MotorController, functions found in motors.hpp

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void processGamepad(ControllerPtr ctl) {
    uint8_t dpad = ctl->dpad();

    if (dpad & DPAD_UP) {
        Serial.println("Moving Forward!");
        robotMotors.forward(255);
    } else if (dpad & DPAD_DOWN) {
        Serial.println("Moving Backward!");
        robotMotors.backward(255);
    } else if (dpad & DPAD_LEFT) {
        Serial.println("Turning Left!");
        robotMotors.turnLeft(255);
    } else if (dpad & DPAD_RIGHT) {
        Serial.println("Turning Right!");
        robotMotors.turnRight(255);
    } else {
        robotMotors.stop();
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize Bluepad32 Bluetooth Stack
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); 
    esp_log_level_set("gpio", ESP_LOG_ERROR); // Suppress log spam
    
    // Allow any controller to connect
    uni_bt_allowlist_set_enabled(false);

    // Initialize motor pins
    robotMotors.init();

    Serial.println("Robot ready! Press DPAD UP to drive forward.");
}

void loop() {
    vTaskDelay(1);
    BP32.update();

    bool controllerConnected = false;

    for (auto myController : myControllers) {
        if (myController && myController->isConnected()) {
            controllerConnected = true;

            if (myController->hasData()) {
                processGamepad(myController);
            }
        }
    }

    // Stop when the controller disconnects.
    if (!controllerConnected) {
        robotMotors.stop();
    }
}