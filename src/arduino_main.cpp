// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <uni.h>
#include "controller_callbacks.h"
#include "autonomous.hpp"
#include "colorSensor.hpp"
#include "motors.hpp"
#include "servo.hpp"

// Motor setup using uPesy ESP32 pins
const int IN1 = 16;
const int IN2 = 17;
const int IN3 = 18;
const int IN4 = 19;
const int LINE_SENSOR_PIN_1 = 32;
const int LINE_SENSOR_PIN_2 = 33;
const int SERVO_PIN = 23;
MotorController robotMotors(IN1, IN2, IN3, IN4); //makes an object of MotorController, functions found in motors.hpp
ColorSensor colorSensor;
AutonomousController autonomousController(colorSensor, robotMotors);
ServoController armServo;
bool colorSensorReady = false;
bool servoReady = false;
bool servoButtonWasPressed[BP32_MAX_GAMEPADS] = {};
bool modeButtonWasPressed[BP32_MAX_GAMEPADS] = {};

enum class DriveMode {
    Autonomous,
    Manual
};

DriveMode driveMode = DriveMode::Autonomous;

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];
/*
void testLineSensors() {
    static unsigned long lastReadingMs = 0;
    if (millis() - lastReadingMs < 250) {
        return;
    }

    lastReadingMs = millis();
    int sensor1Reading = analogRead(LINE_SENSOR_PIN_1);
    int sensor2Reading = analogRead(LINE_SENSOR_PIN_2);
    Serial.printf("Line sensors: GPIO32=%d GPIO33=%d\n",
                  sensor1Reading, sensor2Reading);
}*/

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
        robotMotors.driveJoystick(x / 2, y / 2);
    }
    else if(x < -125 || x > 125)
    {
        robotMotors.driveJoystick(x, y);
    }

}

void setup() {
    Serial.begin(115200);

    colorSensorReady = colorSensor.begin();
    if (!colorSensorReady) {
        Serial.println("Failed to initialize color sensor; autonomous mode is unavailable.");
    }

    // Initialize Bluepad32 Bluetooth Stack
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); 
    esp_log_level_set("gpio", ESP_LOG_ERROR); // Suppress log spam
    
    // Allow any controller to connect
    uni_bt_allowlist_set_enabled(false);

    // Initialize motor pins
    robotMotors.init();

    servoReady = armServo.begin(SERVO_PIN);
    if (!servoReady) {
        Serial.println("Failed to initialize servo.");
    }

    pinMode(LINE_SENSOR_PIN_1, INPUT);
    pinMode(LINE_SENSOR_PIN_2, INPUT);

    autonomousController.begin();
    Serial.println("Robot ready in autonomous mode. Press Start to switch drive mode.");
}

void loop() {
    vTaskDelay(1);
    BP32.update();
    armServo.update();
    //testLineSensors();

    bool controllerConnected = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
        ControllerPtr myController = myControllers[i];
        if (myController && myController->isConnected()) {
            controllerConnected = true;

            bool servoButtonPressed = myController->a();
            if (servoReady && servoButtonPressed && !servoButtonWasPressed[i]) {
                armServo.toggle();
            }
            servoButtonWasPressed[i] = servoButtonPressed;

            bool modeButtonPressed = myController->b();
            if (modeButtonPressed && !modeButtonWasPressed[i]) { //the array stores the previous value of the button press
                // so we only toggle the drive mode when the button is newly pressed, this code loop will all run super fast
                // so it's getting the new data fast enough that we have to double check that it's actually a new press by analyzing previous
                driveMode = driveMode == DriveMode::Autonomous
                    ? DriveMode::Manual
                    : DriveMode::Autonomous;
                robotMotors.stop();
                Serial.println(driveMode == DriveMode::Autonomous
                    ? "Autonomous mode"
                    : "Manual mode");
            }
            modeButtonWasPressed[i] = modeButtonPressed;

            if (driveMode == DriveMode::Manual && myController->hasData()) {
                processGamepad(myController);
            }
        } else {
            servoButtonWasPressed[i] = false;
            modeButtonWasPressed[i] = false;
        }
    }

    if (driveMode == DriveMode::Autonomous) {
        if (colorSensorReady) {
            autonomousController.update();
        } else {
            robotMotors.stop();
        }
    } else if (!controllerConnected) {
        robotMotors.stop();
    }
}