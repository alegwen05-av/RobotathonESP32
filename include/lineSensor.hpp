#pragma once

#include <stdint.h>

class LineSensor {
public:
    static constexpr uint8_t Count = 3;

    // Configures the ADC pins and automatically starts calibration.
    void begin();

    // Restarts the eight-second white/black calibration process.
    void startCalibration();

    // Samples the sensors and updates line detection and steering.
    void update();

    // Clears steering and lost-line recovery history.
    void resetSteering();

    // Prints calibration and live sensor diagnostics over Serial.
    void print();

    bool ready() const {
        return ready_;
    }

    bool calibrating() const {
        return calibrating_;
    }

    bool detected() const {
        return detected_;
    }

    // Negative steers left, positive steers right.
    int correction() const {
        return correction_;
    }

private:
    uint16_t raw_[Count] = {};
    uint16_t whiteStrength_[Count] = {};
    uint16_t minimum_[Count] = {};
    uint16_t maximum_[Count] = {};

    bool ready_ = false;
    bool calibrating_ = false;
    bool detected_ = false;

    uint32_t calibrationStartMs_ = 0;
    uint32_t lastReadMs_ = 0;

    int correction_ = 0;
    float previousError_ = 0.0f;
    bool previousErrorValid_ = false;

    int8_t lastDirection_ = 0;
    uint32_t lastLineMs_ = 0;
    bool hasSeenLine_ = false;
};