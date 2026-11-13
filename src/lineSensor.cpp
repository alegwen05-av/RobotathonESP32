#include "lineSensor.hpp"

namespace {

// Physical order: LEFT, MIDDLE, RIGHT.
const uint8_t pins[LineSensor::Count] = {32, 34, 33};

constexpr uint32_t CALIBRATION_MS = 8000;
constexpr uint32_t SAMPLE_MS = 10;

constexpr int MINIMUM_CALIBRATION_SPAN = 80;

// White strength: 0 = black, 1000 = white.
constexpr int WHITE_ON_THRESHOLD = 300;
constexpr int WHITE_OFF_THRESHOLD = 180;

// Normal steering.
constexpr int TURN_CORRECTION = 50;

// Brief search toward the last known line direction.
constexpr uint32_t RECOVERY_TIMEOUT_MS = 600;
constexpr int RECOVERY_CORRECTION = 220;

}

void LineSensor::begin() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    for (uint8_t pin : pins) {
        pinMode(pin, INPUT);
    }

    // Leave QTR-8A LEDON disconnected to keep emitters on.
    startCalibration();
}

void LineSensor::startCalibration() {
    ready_ = false;
    calibrating_ = true;
    detected_ = false;

    calibrationStart_ = millis();
    lastRead_ = calibrationStart_;

    resetSteering();

    for (int i = 0; i < Count; ++i) {
        low_[i] = 4095;
        high_[i] = 0;
        raw_[i] = 0;
        white_[i] = 0;
        onWhite_[i] = false;
    }

    Serial.println(
        "Calibration: sweep ALL THREE sensors across "
        "white and black for 8 seconds."
    );
}

void LineSensor::resetSteering() {
    correction_ = 0;
    lastDirection_ = 0;
    lastLineMs_ = 0;
    hasSeenLine_ = false;
}

void LineSensor::update() {
    const uint32_t now = millis();

    if (now - lastRead_ < SAMPLE_MS) {
        return;
    }

    lastRead_ = now;

    // Read each sensor and average four samples.
    for (int i = 0; i < Count; ++i) {
        uint32_t total = 0;

        for (int sample = 0; sample < 4; ++sample) {
            total += analogRead(pins[i]);
        }

        raw_[i] = total / 4;
    }

    // During calibration, record each sensor's range.
    if (calibrating_) {
        for (int i = 0; i < Count; ++i) {
            if (raw_[i] < low_[i]) {
                low_[i] = raw_[i];
            }

            if (raw_[i] > high_[i]) {
                high_[i] = raw_[i];
            }
        }

        if (now - calibrationStart_ >= CALIBRATION_MS) {
            calibrating_ = false;
            ready_ = true;

            for (int i = 0; i < Count; ++i) {
                Serial.printf(
                    "Sensor %d: min=%u max=%u\n",
                    i,
                    low_[i],
                    high_[i]
                );

                if (high_[i] - low_[i] <
                    MINIMUM_CALIBRATION_SPAN) {
                    ready_ = false;
                }
            }

            Serial.println(
                ready_
                    ? "Line calibration ready."
                    : "Calibration FAILED. Send c to retry."
            );
        }

        return;
    }

    if (!ready_) {
        detected_ = false;
        correction_ = 0;
        return;
    }

    // Convert raw readings to calibrated white strength.
    for (int i = 0; i < Count; ++i) {
        const long strength =
            (long(high_[i]) - long(raw_[i])) * 1000L
            / (long(high_[i]) - long(low_[i]));

        white_[i] = constrain(strength, 0L, 1000L);

        // Separate thresholds reduce flickering.
        if (white_[i] >= WHITE_ON_THRESHOLD) {
            onWhite_[i] = true;
        } else if (white_[i] <= WHITE_OFF_THRESHOLD) {
            onWhite_[i] = false;
        }
    }

    const bool left = onWhite_[0];
    const bool middle = onWhite_[1];
    const bool right = onWhite_[2];

    correction_ = 0;
    detected_ = false;

    // Remember the most recent unambiguous side.
    if (left && !right) {
        lastDirection_ = -1;
    } else if (right && !left) {
        lastDirection_ = 1;
    }

    if (middle) {
        // Middle sees white: both wheels forward equally.
        detected_ = true;
        correction_ = 0;
        lastLineMs_ = now;
        hasSeenLine_ = true;

    } else if (left && !right) {
        // White is to the left: steer left.
        detected_ = true;
        correction_ = -TURN_CORRECTION;
        lastLineMs_ = now;
        hasSeenLine_ = true;

    } else if (right && !left) {
        // White is to the right: steer right.
        detected_ = true;
        correction_ = TURN_CORRECTION;
        lastLineMs_ = now;
        hasSeenLine_ = true;

    } else if (!left && !middle && !right) {
        // All black: briefly search toward the last
        // known side, then stop if white is not found.
        if (hasSeenLine_ &&
            lastDirection_ != 0 &&
            now - lastLineMs_ < RECOVERY_TIMEOUT_MS) {

            correction_ =
                lastDirection_ * RECOVERY_CORRECTION;

            // Allows the existing autonomous controller
            // to drive during recovery.
            detected_ = true;
        }
    }

    // Both sides white with middle black is ambiguous:
    // detected_ remains false, so the car stops.
}

void LineSensor::print() {
    Serial.printf(
        "ready=%d line=%d correction=%d | raw L/M/R:",
        ready_,
        detected_,
        correction_
    );

    for (int i = 0; i < Count; ++i) {
        Serial.printf(" %u", raw_[i]);
    }

    Serial.print(" | white L/M/R:");

    for (int i = 0; i < Count; ++i) {
        Serial.printf(" %u", white_[i]);
    }

    Serial.printf(
        " | detected L/M/R: %d %d %d\n",
        onWhite_[0],
        onWhite_[1],
        onWhite_[2]
    );
}