#include "servo.hpp"

namespace {
constexpr int SERVO_MIN_PULSE_US = 500;
constexpr int SERVO_MAX_PULSE_US = 2500;
constexpr int SERVO_POSITIONS[] = {0, 90, 180, 90};
constexpr uint8_t SERVO_POSITION_COUNT =
    sizeof(SERVO_POSITIONS) / sizeof(SERVO_POSITIONS[0]);
}

bool ServoController::begin(int pin) {
    if (servo_.attach(pin, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US) == 0) {
        return false;
    }

    positionIndex_ = 0;
    servo_.write(SERVO_POSITIONS[positionIndex_]);
    return true;
}

void ServoController::advancePosition() {
    positionIndex_ = (positionIndex_ + 1) % SERVO_POSITION_COUNT;
    servo_.write(SERVO_POSITIONS[positionIndex_]);
}