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
    int x = ctl->axisX();
    int y = ctl->axisY();
    constexpr int deadzone = 0;

    if (x > -deadzone && x < deadzone) {
        x = 0;
    }
    if (y > -deadzone && y < deadzone) {
        y = 0;
    }
    if(x > -125 && x < 125) //maual code, meant to make the robot drive slower when the stick is near the center, and faster when the stick is pushed further out
    {
        robotMotors.driveJoystick(x / 2, -y / 2);
    }
    else if(x < -125 || x > 125)
    {
        robotMotors.driveJoystick(x, -y);
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

    Serial.println("Robot ready! Use the left stick to drive.");
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