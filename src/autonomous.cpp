#include "autonomous.hpp"
#include <Arduino.h>
#include "motors.hpp"

namespace {
constexpr unsigned long COLOR_POLL_INTERVAL_MS = 100;
constexpr uint8_t COLOR_CONFIRMATION_READS = 2;
constexpr int AUTONOMOUS_FORWARD_SPEED = 120;

const char* colorName(DetectedColor color) {
    switch (color) {
        case DetectedColor::Red: return "red";
        case DetectedColor::Orange: return "orange";
        case DetectedColor::Indigo: return "indigo";
        default: return "unknown";
    }
}
}

AutonomousController::AutonomousController(ColorSensor& colorSensor, MotorController& motors)
    : colorSensor_(colorSensor), motors_(motors) {}

void AutonomousController::begin() {
    state_ = State::LearningStartColor;
    startColor_ = DetectedColor::None;
    candidateColor_ = DetectedColor::None;
    candidateReadCount_ = 0;
    lastReadingMs_ = millis();
    motors_.stop();
}

void AutonomousController::update() {
    if (state_ == State::Stopped || millis() - lastReadingMs_ < COLOR_POLL_INTERVAL_MS) {
        return;
    }

    lastReadingMs_ = millis();
    ColorReading reading{};
    if (!colorSensor_.read(reading)) {
        return;
    }

    const DetectedColor observedColor = colorSensor_.classify(reading);
    Serial.printf("Color: R=%d G=%d B=%d C=%d (%s)",
                  reading.red, reading.green, reading.blue, reading.clear,
                  colorName(observedColor));
    Serial.println();

    if (observedColor == DetectedColor::None) {
        candidateColor_ = DetectedColor::None;
        candidateReadCount_ = 0;
        return;
    }

    if (observedColor == candidateColor_) {
        if (candidateReadCount_ < COLOR_CONFIRMATION_READS) {
            ++candidateReadCount_;
        }
    } else {
        candidateColor_ = observedColor;
        candidateReadCount_ = 1;
    }

    if (candidateReadCount_ < COLOR_CONFIRMATION_READS) {
        return;
    }

    switch (state_) {
        case State::LearningStartColor:
            startColor_ = observedColor;
            state_ = State::LeavingStartTile;
            motors_.forward(AUTONOMOUS_FORWARD_SPEED);
            Serial.print("Start color: ");
            Serial.print(colorName(startColor_));
            Serial.println("; moving forward.");
            break;
        case State::LeavingStartTile:
            if (observedColor != startColor_) {
                state_ = State::SeekingStartColor;
                Serial.println("Left start tile; seeking it again.");
            }
            break;
        case State::SeekingStartColor:
            if (observedColor == startColor_) {
                motors_.stop();
                state_ = State::Stopped;
                Serial.println("Start color found again; stopped.");
            }
            break;
        case State::Stopped:
            break;
    }
}
