#include "autonomous.hpp"
#include <Arduino.h>
#include "lineSensor.hpp"
#include "motors.hpp"

namespace {
constexpr int COLOR_SPEED = 100;
constexpr int LINE_SPEED = 180;
constexpr uint32_t COLOR_INTERVAL_MS = 100;
constexpr uint32_t COLOR_MOVE_DURATION_MS = 500;
constexpr uint32_t COLOR_SCAN_PAUSE_MS = 500;
constexpr uint32_t COLOR_REPORT_INTERVAL_MS = 250;
constexpr uint32_t MESSAGE_INTERVAL_MS = 1000;
constexpr uint8_t CONFIRMATION_READS = 2;
constexpr float MATCH_DISTANCE_SQUARED = 0.0144f;
constexpr float LEAVE_DISTANCE_SQUARED = 0.0225f;
constexpr int MINIMUM_RGB_TOTAL = 10;

const char* detectedColorName(DetectedColor color) {
    switch (color) {
        case DetectedColor::Red:
            return "red";
        case DetectedColor::Orange:
            return "orange";
        case DetectedColor::Green:
            return "green";
        case DetectedColor::Indigo:
            return "indigo";
        case DetectedColor::None:
        default:
            return "none";
    }
}
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
    colorMotionPhase_ = ColorMotionPhase::Scan;
    confirmationCount_ = 0;
    startBrightness_ = 0.0f;
    startDetectedColor_ = DetectedColor::None;
    lastColorMs_ = millis();
    colorMotionPhaseStartMs_ = lastColorMs_;
    lastColorReportMs_ = lastColorMs_;
    lastMessageMs_ = millis();
    lineSensor_.resetSteering();
}

void AutonomousController::startColorChallenge() {
    motors_.stop();
    lineSensor_.resetSteering();
    challenge_ = Challenge::Color;
    colorState_ = ColorState::CaptureStart;
    colorMotionPhase_ = ColorMotionPhase::Scan;
    confirmationCount_ = 0;
    startBrightness_ = 0.0f;
    startDetectedColor_ = DetectedColor::None;
    lastColorMs_ = millis();
    colorMotionPhaseStartMs_ = lastColorMs_;
    lastColorReportMs_ = lastColorMs_;
    lastMessageMs_ = millis();
    Serial.println("COLOR SELECTED: capturing the starting tile.");
}

void AutonomousController::startLineChallenge() {
    motors_.stop();
    lineSensor_.resetSteering();
    lineSensor_.startCalibration();
    challenge_ = Challenge::Line;
    Serial.println("LINE SELECTED: calibrating sensors; line following starts when calibration completes.");
}

void AutonomousController::pause() {
    motors_.stop();
    lineSensor_.resetSteering();
    confirmationCount_ = 0;
    lastColorMs_ = millis();
    colorMotionPhase_ = ColorMotionPhase::Scan;
    colorMotionPhaseStartMs_ = lastColorMs_;
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
    if (colorState_ != ColorState::CaptureStart) {
        if (colorMotionPhase_ == ColorMotionPhase::Move) {
            if (now - colorMotionPhaseStartMs_ < COLOR_MOVE_DURATION_MS) {
                motors_.forward(COLOR_SPEED);
                return;
            }

            colorMotionPhase_ = ColorMotionPhase::Scan;
            colorMotionPhaseStartMs_ = now;
            lastColorMs_ = now;
        } else if (now - colorMotionPhaseStartMs_ >= COLOR_SCAN_PAUSE_MS) {
            colorMotionPhase_ = ColorMotionPhase::Move;
            colorMotionPhaseStartMs_ = now;
            lastColorMs_ = now;
            motors_.forward(COLOR_SPEED);
            return;
        }
    }

    if (now - lastColorMs_ >= COLOR_INTERVAL_MS) {
        lastColorMs_ = now;
        ColorReading reading{};

        if (!colorSensor_.read(reading)) {
            if (now - lastColorReportMs_ >= COLOR_REPORT_INTERVAL_MS) {
                lastColorReportMs_ = now;
                Serial.println("COLOR sensor is waiting for a reading.");
            }
        } else {
            const DetectedColor detectedColor = colorSensor_.classify(reading);
            if (colorState_ == ColorState::CaptureStart &&
                now - lastColorReportMs_ >= COLOR_REPORT_INTERVAL_MS) {
                lastColorReportMs_ = now;
                Serial.printf(
                    "COLOR state=capture-start RGB=%d/%d/%d clear=%d class=%s\n",
                    reading.red,
                    reading.green,
                    reading.blue,
                    reading.clear,
                    detectedColorName(detectedColor)
                );
            }

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
                    startDetectedColor_ = colorSensor_.classify(reading);
                    colorState_ = ColorState::LeaveStart;
                    colorMotionPhase_ = ColorMotionPhase::Move;
                    colorMotionPhaseStartMs_ = now;
                    confirmationCount_ = 0;
                    Serial.printf(
                        "START SAVED: R=%d G=%d B=%d class=%s. Moving to find a different color.\n",
                        reading.red,
                        reading.green,
                        reading.blue,
                        detectedColorName(startDetectedColor_)
                    );
                } else if (now - lastMessageMs_ >= MESSAGE_INTERVAL_MS) {
                    lastMessageMs_ = now;
                    Serial.println("COLOR: waiting for a usable tile reading.");
                }
            } else {
                bool matchesStart = false;
                bool differentFromStart = false;

                if (total < MINIMUM_RGB_TOTAL) {
                    differentFromStart = true;
                    if (now - lastColorReportMs_ >= COLOR_REPORT_INTERVAL_MS) {
                        lastColorReportMs_ = now;
                        Serial.printf(
                            "COLOR state=%s reading-too-dim RGB=%d/%d/%d\n",
                            colorState_ == ColorState::LeaveStart
                                ? "leave-start"
                                : "find-start-again",
                            reading.red,
                            reading.green,
                            reading.blue
                        );
                    }
                } else {
                    const float red = reading.red / total;
                    const float green = reading.green / total;
                    const float blue = reading.blue / total;
                    const float dr = red - startRed_;
                    const float dg = green - startGreen_;
                    const float db = blue - startBlue_;
                    const float distanceSquared = dr * dr + dg * dg + db * db;
                    const float brightnessRatio = total / startBrightness_;
                    const bool distinctClass =
                        startDetectedColor_ != DetectedColor::None &&
                        detectedColor != DetectedColor::None &&
                        startDetectedColor_ != detectedColor;
                    const bool colorChanged = distinctClass ||
                        ((startDetectedColor_ == DetectedColor::None ||
                          detectedColor == DetectedColor::None) &&
                         distanceSquared >= LEAVE_DISTANCE_SQUARED);

                    const bool sameClass =
                        startDetectedColor_ != DetectedColor::None &&
                        detectedColor == startDetectedColor_;
                    const bool sameColor =
                        startDetectedColor_ != DetectedColor::None
                            ? sameClass
                            : distanceSquared <= MATCH_DISTANCE_SQUARED;
                    matchesStart = sameColor &&
                        brightnessRatio >= 0.35f && brightnessRatio <= 2.80f;
                    differentFromStart = colorChanged ||
                        brightnessRatio < 0.35f || brightnessRatio > 2.80f;

                    if (now - lastColorReportMs_ >= COLOR_REPORT_INTERVAL_MS) {
                        lastColorReportMs_ = now;
                        Serial.printf(
                            "COLOR state=%s current=%s start=%s dist2=%.4f brightness=%.2f changed=%d match=%d\n",
                            colorState_ == ColorState::LeaveStart
                                ? "leave-start"
                                : "find-start-again",
                            detectedColorName(detectedColor),
                            detectedColorName(startDetectedColor_),
                            distanceSquared,
                            brightnessRatio,
                            differentFromStart,
                            matchesStart
                        );
                    }
                }

                if (colorState_ == ColorState::LeaveStart) {
                    confirmationCount_ = differentFromStart ? confirmationCount_ + 1 : 0;
                    if (confirmationCount_ >= CONFIRMATION_READS) {
                        confirmationCount_ = 0;
                        colorState_ = ColorState::FindStartAgain;
                        Serial.printf(
                            "LEFT START TILE: detected %s; continuing forward.\n",
                            detectedColorName(detectedColor)
                        );
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

    if (colorState_ != ColorState::CaptureStart &&
        colorMotionPhase_ == ColorMotionPhase::Move) {
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
