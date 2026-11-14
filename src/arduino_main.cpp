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
bool colorButtonWasPressed[BP32_MAX_GAMEPADS] = {};

enum class DriveMode {
    Autonomous,
    Manual
};

DriveMode driveMode = DriveMode::Manual;

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];

namespace {
constexpr uint32_t SENSOR_REPORT_INTERVAL_MS = 500;
uint32_t lastLineReportMs = 0;

void handleSerialInput() {
    while (Serial.available() > 0) {
        const char command = static_cast<char>(Serial.read());
        if (command == 'c' || command == 'C') {
            driveMode = DriveMode::Autonomous;
            robotMotors.stop();
            autonomousController.startLineChallenge();
            lastLineReportMs = 0;
        }
    }
}
}

void processGamepad(ControllerPtr ctl) {
    constexpr int FORWARD_SPEED = 180;
    constexpr int TURN_START_SPEED = 100;
    constexpr int TURN_MAX_SPEED = 200;
    constexpr unsigned long TURN_RAMP_TIME_MS = 1500;

    static int previousTurnDirection = 0;
    static unsigned long turnStartedMs = 0;

    const int dpad = ctl->dpad();

    int forwardSpeed = 0;
    if ((dpad & DPAD_UP) && !(dpad & DPAD_DOWN)) {
        forwardSpeed = FORWARD_SPEED;
    } else if ((dpad & DPAD_DOWN) && !(dpad & DPAD_UP)) {
        forwardSpeed = -FORWARD_SPEED;
    }

    int turnDirection = 0;
    if ((dpad & DPAD_LEFT) && !(dpad & DPAD_RIGHT)) {
        turnDirection = -1;
    } else if ((dpad & DPAD_RIGHT) && !(dpad & DPAD_LEFT)) {
        turnDirection = 1;
    }

    int turnSpeed = 0;
    if (turnDirection != 0) {
        if (turnDirection != previousTurnDirection) {
            turnStartedMs = millis();
            previousTurnDirection = turnDirection;
        }

        unsigned long turnDuration = millis() - turnStartedMs;
        if (turnDuration > TURN_RAMP_TIME_MS) {
            turnDuration = TURN_RAMP_TIME_MS;
        }

        turnSpeed = TURN_START_SPEED +
            (TURN_MAX_SPEED - TURN_START_SPEED) *
            turnDuration / TURN_RAMP_TIME_MS;
    } else {
        previousTurnDirection = 0;
    }

    const int x = turnDirection * turnSpeed;
    if (forwardSpeed == 0 && x == 0) {
        robotMotors.stop();
    } else {
        robotMotors.driveJoystick(x, forwardSpeed);
    }
}

void setup() {
    Serial.begin(115200);
    colorSensorReady = colorSensor.begin();

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    uni_bt_allowlist_set_enabled(false);

    robotMotors.init();
    lineSensor.begin();
    Serial.println("Serial diagnostics active at 115200 baud.");
    Serial.println("MANUAL MODE. Press B for automatic color challenge; Y for line or X for color.");

    servoReady = armServo.begin(SERVO_PIN);
    if (!servoReady) {
        Serial.println("Servo initialization failed.");
    }

    autonomousController.begin();
}

void loop() {
    vTaskDelay(1);
    BP32.update();
    handleSerialInput();
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
                robotMotors.stop();
                if (driveMode == DriveMode::Autonomous) {
                    autonomousController.startColorChallenge();
                    Serial.println(colorSensorReady
                        ? "AUTOMATIC COLOR CHALLENGE selected."
                        : "AUTOMATIC COLOR CHALLENGE selected, but the color sensor is unavailable.");
                } else {
                    autonomousController.pause();
                    Serial.println("MANUAL MODE selected.");
                }
            }
            modeButtonWasPressed[i] = modePressed;

            const bool linePressed = controller->y();
            if (linePressed && !lineButtonWasPressed[i]) {
                driveMode = DriveMode::Autonomous;
                robotMotors.stop();
                autonomousController.startLineChallenge();
                lastLineReportMs = 0;
            }
            lineButtonWasPressed[i] = linePressed;

            const bool colorPressed = controller->x();
            if (colorPressed && !colorButtonWasPressed[i]) {
                driveMode = DriveMode::Autonomous;
                robotMotors.stop();
                autonomousController.startColorChallenge();
            }
            colorButtonWasPressed[i] = colorPressed;

            if (driveMode == DriveMode::Manual && controller->hasData()) {
                processGamepad(controller);
            }
        } else {
            servoButtonWasPressed[i] = false;
            modeButtonWasPressed[i] = false;
            lineButtonWasPressed[i] = false;
            colorButtonWasPressed[i] = false;
        }
    }

    if (driveMode == DriveMode::Autonomous &&
        autonomousController.challenge() == AutonomousController::Challenge::Line) {
        lineSensor.update();
    }

    if (driveMode == DriveMode::Autonomous) {
        autonomousController.update(colorSensorReady);
    } else if (!controllerConnected) {
        robotMotors.stop();
    }

    const uint32_t now = millis();
    if (driveMode == DriveMode::Autonomous &&
        autonomousController.challenge() == AutonomousController::Challenge::Line &&
        now - lastLineReportMs >= SENSOR_REPORT_INTERVAL_MS) {
        lastLineReportMs = now;
        lineSensor.print();
    }
}
