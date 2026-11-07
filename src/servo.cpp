#include "servo.hpp"

namespace {
constexpr int SERVO_NEUTRAL_US = 1500;
constexpr int SERVO_UP_US = 1700;
constexpr int SERVO_DOWN_US = 1300;
constexpr unsigned long SERVO_MOVE_DURATION_MS = 200;
}

bool ServoController::begin(int pin) {
    if (servo_.attach(pin) == 0) {
        return false;
    }

    raised_ = false;
    moving_ = false;
    servo_.writeMicroseconds(SERVO_NEUTRAL_US);
    return true;
}

void ServoController::toggle() {
    if (moving_) {
        return;
    }

    raised_ = !raised_;
    servo_.writeMicroseconds(raised_ ? SERVO_UP_US : SERVO_DOWN_US);
    movementStartedMs_ = millis();
    moving_ = true;
}

void ServoController::update() {
    if (moving_ && millis() - movementStartedMs_ >= SERVO_MOVE_DURATION_MS) {
        servo_.writeMicroseconds(SERVO_NEUTRAL_US);
        moving_ = false;
    }
}