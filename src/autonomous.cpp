#include "autonomous.hpp"
#include <Arduino.h>
#include "lineSensor.hpp"
#include "motors.hpp"

namespace {
constexpr int COLOR_SPEED = 180;
constexpr int LINE_SPEED = 180;
constexpr uint32_t COLOR_INTERVAL_MS = 100;
constexpr uint32_t MESSAGE_INTERVAL_MS = 1000;
constexpr uint8_t CONFIRMATION_READS = 3;
constexpr float MATCH_DISTANCE_SQUARED = 0.0144f;
constexpr float LEAVE_DISTANCE_SQUARED = 0.0400f;
constexpr int MINIMUM_RGB_TOTAL = 10;
}

AutonomousController::AutonomousController(
    ColorSensor& colorSensor,
    MotorController& motors,
    LineSensor& lineSensor
)
    : colorSensor_(colorSensor), motors_(motors), lineSensor_(lineSensor) {}

void AutonomousController::begin() {
    motors_.stop();
    challenge_ = Challenge::None;
    colorState_ = ColorState::CaptureStart;
    confirmationCount_ = 0;
    startBrightness_ = 0.0f;
    lastColorMs_ = millis();
    lastMessageMs_ = millis();
    lineSensor_.resetSteering();
}

void AutonomousController::startColorChallenge() {
    motors_.stop();
    lineSensor_.resetSteering();
    challenge_ = Challenge::Color;
    colorState_ = ColorState::CaptureStart;
    confirmationCount_ = 0;
    startBrightness_ = 0.0f;
    lastColorMs_ = millis();
    lastMessageMs_ = millis();
    Serial.println("COLOR SELECTED: capturing the starting tile.");
}

void AutonomousController::startLineChallenge() {
    motors_.stop();
    lineSensor_.resetSteering();
    challenge_ = Challenge::Line;
    Serial.println("LINE SELECTED: following the white line.");
    if (!lineSensor_.ready()) {
        Serial.println("Line calibration not ready. Send c to calibrate.");
    }
}

void AutonomousController::pause() {
    motors_.stop();
    lineSensor_.resetSteering();
    confirmationCount_ = 0;
    lastColorMs_ = millis();
}

void AutonomousController::update(bool colorSensorReady) {
    switch (challenge_) {
        case Challenge::Color:
            if (colorSensorReady) {
                updateColor();
            } else {
                motors_.stop();
                if (millis() - lastMessageMs_ >= MESSAGE_INTERVAL_MS) {
                    lastMessageMs_ = millis();
                    Serial.println("COLOR BLOCKED: color sensor is unavailable.");
                }
            }
            break;
        case Challenge::Line:
            updateLine();
            break;
        case Challenge::None:
        default:
            motors_.stop();
            break;
    }
}

void AutonomousController::updateColor() {
    if (colorState_ == ColorState::Finished) {
        motors_.stop();
        return;
    }

    const uint32_t now = millis();
    if (now - lastColorMs_ >= COLOR_INTERVAL_MS) {
        lastColorMs_ = now;
        ColorReading reading{};

        if (!colorSensor_.read(reading)) {
            confirmationCount_ = 0;
        } else {
            const float total = static_cast<float>(reading.red) +
                static_cast<float>(reading.green) +
                static_cast<float>(reading.blue);

            if (colorState_ == ColorState::CaptureStart) {
                if (total >= MINIMUM_RGB_TOTAL && reading.red >= 0 &&
                    reading.green >= 0 && reading.blue >= 0) {
                    startRed_ = reading.red / total;
                    startGreen_ = reading.green / total;
                    startBlue_ = reading.blue / total;
                    startBrightness_ = total;
                    colorState_ = ColorState::LeaveStart;
                    confirmationCount_ = 0;
                    Serial.printf("START SAVED: R=%d G=%d B=%d. Driving forward.\n",
                        reading.red, reading.green, reading.blue);
                } else if (now - lastMessageMs_ >= MESSAGE_INTERVAL_MS) {
                    lastMessageMs_ = now;
                    Serial.println("COLOR: waiting for a usable tile reading.");
                }
            } else {
                bool matchesStart = false;
                bool differentFromStart = false;

                if (total < MINIMUM_RGB_TOTAL) {
                    differentFromStart = true;
                } else {
                    const float red = reading.red / total;
                    const float green = reading.green / total;
                    const float blue = reading.blue / total;
                    const float dr = red - startRed_;
                    const float dg = green - startGreen_;
                    const float db = blue - startBlue_;
                    const float distanceSquared = dr * dr + dg * dg + db * db;
                    const float brightnessRatio = total / startBrightness_;

                    matchesStart = distanceSquared <= MATCH_DISTANCE_SQUARED &&
                        brightnessRatio >= 0.50f && brightnessRatio <= 2.00f;
                    differentFromStart = distanceSquared >= LEAVE_DISTANCE_SQUARED ||
                        brightnessRatio < 0.35f || brightnessRatio > 2.80f;
                }

                if (colorState_ == ColorState::LeaveStart) {
                    confirmationCount_ = differentFromStart ? confirmationCount_ + 1 : 0;
                    if (confirmationCount_ >= CONFIRMATION_READS) {
                        confirmationCount_ = 0;
                        colorState_ = ColorState::FindStartAgain;
                        Serial.println("LEFT START TILE: continuing forward.");
                    }
                } else if (colorState_ == ColorState::FindStartAgain) {
                    confirmationCount_ = matchesStart ? confirmationCount_ + 1 : 0;
                    if (confirmationCount_ >= CONFIRMATION_READS) {
                        confirmationCount_ = 0;
                        colorState_ = ColorState::Finished;
                        Serial.println("START COLOR FOUND AGAIN: stopped.");
                    }
                }
            }
        }
    }

    if (colorState_ == ColorState::LeaveStart ||
        colorState_ == ColorState::FindStartAgain) {
        motors_.forward(COLOR_SPEED);
    } else {
        motors_.stop();
    }
}

void AutonomousController::updateLine() {
    if (!lineSensor_.ready() || !lineSensor_.detected()) {
        motors_.stop();
        return;
    }

    motors_.driveJoystick(lineSensor_.correction(), LINE_SPEED);
}
