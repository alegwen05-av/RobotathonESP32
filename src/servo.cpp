#include "servo.hpp"

namespace {
constexpr int SERVO_MIN_PULSE_US = 500;
constexpr int SERVO_MAX_PULSE_US = 2500;
}

bool ServoController::begin(int pin) {
    if (servo_.attach(pin, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US) == 0) {
        return false;
    }

    atEndPosition_ = false;
    servo_.write(0);
    return true;
}

void ServoController::toggle() {
    atEndPosition_ = !atEndPosition_;
    servo_.write(atEndPosition_ ? 180 : 0);
}