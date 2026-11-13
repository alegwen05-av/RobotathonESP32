#pragma once

#include <Arduino.h>

class LineSensor {
public:
    static constexpr uint8_t Count = 3;

    void begin();
    void startCalibration();
    void update();
    void resetSteering();
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

    int correction() const {
        return correction_;
    }

private:
    uint16_t raw_[Count] = {};
    uint16_t white_[Count] = {};
    uint16_t low_[Count] = {};
    uint16_t high_[Count] = {};

    bool onWhite_[Count] = {};

    bool ready_ = false;
    bool calibrating_ = false;
    bool detected_ = false;

    uint32_t calibrationStart_ = 0;
    uint32_t lastRead_ = 0;

    int correction_ = 0;
    int lastDirection_ = 0;  // -1 = left, +1 = right
    uint32_t lastLineMs_ = 0;
    bool hasSeenLine_ = false;
};