#pragma once

#include <Arduino.h>
#include "colorSensor.hpp"

class MotorController;
class LineSensor;

class AutonomousController {
public:
    enum class Challenge {
        None,
        Color,
        Line
    };

    AutonomousController(
        ColorSensor& colorSensor,
        MotorController& motors,
        LineSensor& lineSensor
    );

    void begin();
    void startColorChallenge();
    void startLineChallenge();
    void pause();
    void update(bool colorSensorReady);

    Challenge challenge() const {
        return challenge_;
    }

private:
    enum class ColorState {
        CaptureStart,
        LeaveStart,
        FindStartAgain,
        Finished
    };

    enum class ColorMotionPhase {
        Move,
        Scan
    };

    void updateColor();
    void updateLine();

    ColorSensor& colorSensor_;
    MotorController& motors_;
    LineSensor& lineSensor_;

    Challenge challenge_ = Challenge::None;
    ColorState colorState_ = ColorState::CaptureStart;
    ColorMotionPhase colorMotionPhase_ = ColorMotionPhase::Scan;

    float startRed_ = 0.0f;
    float startGreen_ = 0.0f;
    float startBlue_ = 0.0f;
    float startBrightness_ = 0.0f;
    DetectedColor startDetectedColor_ = DetectedColor::None;

    uint8_t confirmationCount_ = 0;

    uint32_t lastColorMs_ = 0;
    uint32_t lastColorReportMs_ = 0;
    uint32_t lastMessageMs_ = 0;
    uint32_t colorMotionPhaseStartMs_ = 0;
};