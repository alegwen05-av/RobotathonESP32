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
#include "lineSensor.hpp"
#include "motors.hpp"
#include "servo.hpp"

const int IN1 = 16;
const int IN2 = 17;
const int IN3 = 18;
const int IN4 = 19;
const int SERVO_PIN = 23;

MotorController robotMotors(IN1, IN2, IN3, IN4);
ColorSensor colorSensor;
LineSensor lineSensor;
AutonomousController autonomousController(colorSensor, robotMotors, lineSensor);
ServoController armServo;

bool colorSensorReady = false;
bool servoReady = false;
bool servoButtonWasPressed[BP32_MAX_GAMEPADS] = {};
bool modeButtonWasPressed[BP32_MAX_GAMEPADS] = {};
bool lineButtonWasPressed[BP32_MAX_GAMEPADS] = {};

enum class DriveMode {
    Autonomous,
    Manual
};

DriveMode driveMode = DriveMode::Autonomous;

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void processGamepad(ControllerPtr controller) {
    int x = controller->axisX();
    int y = controller->axisY();

    if (x > -125 && x < 125) {
        robotMotors.driveJoystick(x / 2, y / 2);
    } else {
        robotMotors.driveJoystick(x, y);
    }
}

void setup() {
    Serial.begin(115200);
    colorSensorReady = colorSensor.begin();
    if (!colorSensorReady) {
        Serial.println("Color sensor initialization failed.");
    }

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    uni_bt_allowlist_set_enabled(false);

    robotMotors.init();
    lineSensor.begin();

    servoReady = armServo.begin(SERVO_PIN);
    if (!servoReady) {
        Serial.println("Servo initialization failed.");
    }

    autonomousController.begin();
    autonomousController.startColorChallenge();
    Serial.println("Color challenge starts automatically. B toggles manual drive; Y selects line following.");
}

void loop() {
    vTaskDelay(1);
    BP32.update();
    lineSensor.update();
    armServo.update();

    bool controllerConnected = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
        ControllerPtr controller = myControllers[i];
        if (controller && controller->isConnected()) {
            controllerConnected = true;

            const bool servoPressed = controller->a();
            if (servoReady && servoPressed && !servoButtonWasPressed[i]) {
                armServo.toggle();
            }
            servoButtonWasPressed[i] = servoPressed;

            const bool modePressed = controller->b();
            if (modePressed && !modeButtonWasPressed[i]) {
                driveMode = driveMode == DriveMode::Autonomous
                    ? DriveMode::Manual
                    : DriveMode::Autonomous;
                if (driveMode == DriveMode::Manual) {
                    autonomousController.pause();
                }
                robotMotors.stop();
                Serial.println(driveMode == DriveMode::Autonomous
                    ? "Autonomous mode"
                    : "Manual mode");
            }
            modeButtonWasPressed[i] = modePressed;

            const bool linePressed = controller->y();
            if (driveMode == DriveMode::Autonomous && linePressed &&
                !lineButtonWasPressed[i]) {
                autonomousController.startLineChallenge();
            }
            lineButtonWasPressed[i] = linePressed;

            if (driveMode == DriveMode::Manual && controller->hasData()) {
                processGamepad(controller);
            }
        } else {
            servoButtonWasPressed[i] = false;
            modeButtonWasPressed[i] = false;
            lineButtonWasPressed[i] = false;
        }
    }

    if (driveMode == DriveMode::Autonomous) {
        autonomousController.update(colorSensorReady);
    } else if (!controllerConnected) {
        robotMotors.stop();
    }
}
