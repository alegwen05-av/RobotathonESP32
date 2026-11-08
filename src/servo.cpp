#include "servo.hpp"

namespace {
constexpr int SERVO_STOP_US = 1500;
constexpr int SERVO_FORWARD_US = 1900;
constexpr int SERVO_REVERSE_US = 900;
constexpr unsigned long SERVO_PHASE_DURATION_MS = 500;
}

bool ServoController::begin(int pin) {
    if (servo_.attach(pin) == 0) {
        return false;
    }

    moving_ = false;
    forwardNext_ = true;
    servo_.writeMicroseconds(SERVO_STOP_US);
    return true;
}

void ServoController::toggle() {
    if (moving_) {
        return;
    }

    moving_ = true;
    phaseStartedMs_ = millis();
    servo_.writeMicroseconds(forwardNext_ ? SERVO_FORWARD_US : SERVO_REVERSE_US);
    forwardNext_ = !forwardNext_;
}

void ServoController::update() {
    if (!moving_ || millis() - phaseStartedMs_ < SERVO_PHASE_DURATION_MS) {
        return;
    }

    moving_ = false;
    servo_.writeMicroseconds(SERVO_STOP_US);
}