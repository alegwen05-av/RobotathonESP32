#include "../include/lineSensor.hpp"
#include <Arduino.h>

namespace {

// Sensor order must match the physical left-to-right placement on the robot.
const uint8_t SENSOR_PINS[LineSensor::Count] = {32, 33, 34};
constexpr uint8_t STATUS_LED_PIN = 2;

constexpr uint32_t CALIBRATION_TIME_MS = 8000;
constexpr uint32_t CALIBRATION_FLASH_MS = 150;
constexpr uint32_t SAMPLE_INTERVAL_MS = 10;
constexpr uint32_t MAX_DERIVATIVE_GAP_MS = 100;

// Each sensor must see enough difference between the line and the floor.
constexpr uint16_t MINIMUM_CALIBRATION_SPAN = 80;

// The current hardware is expected to produce a lower ADC reading over white.
// Change this to false if diagnostics show that white produces a higher value.
constexpr bool WHITE_READS_LOW = true;

// Normalized sensor strengths run from 0 (black) to 1000 (white).
constexpr int NOISE_FLOOR = 100;
constexpr int LINE_DETECTION_THRESHOLD = 350;

// Steering controller. Sensor positions are -1 (left), 0 (middle), +1 (right).
constexpr float KP = 75.0f;
constexpr float KD = 0.08f;
constexpr int MAX_CORRECTION = 100;

// If the line briefly disappears, continue turning toward its last known side.
constexpr uint32_t RECOVERY_TIME_MS = 400;
constexpr int RECOVERY_CORRECTION = 85;

} // namespace

void LineSensor::begin() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    for (uint8_t pin : SENSOR_PINS) {
        pinMode(pin, INPUT);
    }
    // Autonomous mode requires ready() to be true. Starting calibration here
    // ensures the sensor can actually become ready after power-up.
    startCalibration();
}

void LineSensor::startCalibration() {
    ready_ = false;
    calibrating_ = true;
    detected_ = false;

    calibrationStartMs_ = millis();
    lastReadMs_ = calibrationStartMs_;

    for (uint8_t i = 0; i < Count; ++i) {
        minimum_[i] = 4095;
        maximum_[i] = 0;
        raw_[i] = 0;
        whiteStrength_[i] = 0;
    }

    resetSteering();

    Serial.println(
        "Line calibration started. Sweep LEFT, MIDDLE, and RIGHT sensors "
        "across both white and black for 8 seconds."
    );
}

void LineSensor::resetSteering() {
    correction_ = 0;
    previousError_ = 0.0f;
    previousErrorValid_ = false;
    lastDirection_ = 0;
    lastLineMs_ = 0;
    hasSeenLine_ = false;
}

void LineSensor::update() {
    const uint32_t now = millis();
    const uint32_t elapsed = now - lastReadMs_;

    if (elapsed < SAMPLE_INTERVAL_MS) {
        return;
    }
    lastReadMs_ = now;

    // Average four readings to reduce ADC noise.
    for (uint8_t i = 0; i < Count; ++i) {
        uint32_t sum = 0;
        for (uint8_t sample = 0; sample < 4; ++sample) {
            sum += analogRead(SENSOR_PINS[i]);
        }
        raw_[i] = sum / 4;
    }

    if (calibrating_) {
        for (uint8_t i = 0; i < Count; ++i) {
            minimum_[i] = min(minimum_[i], raw_[i]);
            maximum_[i] = max(maximum_[i], raw_[i]);
        }

        if (now - calibrationStartMs_ < CALIBRATION_TIME_MS) {
            return;
        }

        calibrating_ = false;
        ready_ = false;

        for (uint8_t i = 0; i < Count; ++i) {
            const uint16_t span = maximum_[i] - minimum_[i];
            if (span >= MINIMUM_CALIBRATION_SPAN) {
                ready_ = true;
            }

            Serial.printf(
                "Line sensor %u: min=%u max=%u span=%u\n",
                i,
                minimum_[i],
                maximum_[i],
                span
            );
        }

        Serial.println(
            ready_
                ? "Line calibration ready. At least one sensor is usable."
                : "Line calibration FAILED. Sweep at least one sensor over "
                  "white and black, then send c to retry."
        );
        if (ready_) {
            digitalWrite(STATUS_LED_PIN, HIGH);
            delay(CALIBRATION_FLASH_MS);
            digitalWrite(STATUS_LED_PIN, LOW);
        }
        return;
    }

    if (!ready_) {
        detected_ = false;
        correction_ = 0;
        return;
    }

    long totalStrength = 0;
    float weightedPosition = 0.0f;
    uint16_t peakStrength = 0;

    for (uint8_t i = 0; i < Count; ++i) {
        const long span = long(maximum_[i]) - long(minimum_[i]);
        if (span < MINIMUM_CALIBRATION_SPAN) {
            continue;
        }

        // Convert every usable sensor to the same 0-to-1000 white scale.
        long normalized;
        if (WHITE_READS_LOW) {
            normalized =
                (long(maximum_[i]) - long(raw_[i])) * 1000L / span;
        } else {
            normalized =
                (long(raw_[i]) - long(minimum_[i])) * 1000L / span;
        }

        whiteStrength_[i] =
            uint16_t(constrain(normalized, 0L, 1000L));
        peakStrength = max(peakStrength, whiteStrength_[i]);

        // Ignore the weakest part of the signal so background noise does not
        // pull the estimated position away from the actual line.
        const int usableStrength =
            whiteStrength_[i] > NOISE_FLOOR
                ? whiteStrength_[i] - NOISE_FLOOR
                : 0;

        const float sensorPosition = float(i) - 1.0f;
        totalStrength += usableStrength;
        weightedPosition += usableStrength * sensorPosition;
    }

    detected_ =
        peakStrength >= LINE_DETECTION_THRESHOLD && totalStrength > 0;

    if (detected_) {
        // Negative error means the line is left of center; positive means right.
        const float error = weightedPosition / float(totalStrength);

        float derivative = 0.0f;
        if (previousErrorValid_ && elapsed <= MAX_DERIVATIVE_GAP_MS) {
            derivative =
                (error - previousError_) / (elapsed * 0.001f);
        }

        correction_ = constrain(
            int(KP * error + KD * derivative),
            -MAX_CORRECTION,
            MAX_CORRECTION
        );

        if (error < -0.15f) {
            lastDirection_ = -1;
        } else if (error > 0.15f) {
            lastDirection_ = 1;
        }

        previousError_ = error;
        previousErrorValid_ = true;
        lastLineMs_ = now;
        hasSeenLine_ = true;
        return;
    }

    // Do not carry a derivative measurement across a period with no line.
    previousErrorValid_ = false;

    if (
        hasSeenLine_ &&
        lastDirection_ != 0 &&
        now - lastLineMs_ < RECOVERY_TIME_MS
    ) {
        detected_ = true;
        correction_ = lastDirection_ * RECOVERY_CORRECTION;
    } else {
        correction_ = 0;
    }
}

void LineSensor::print() {
    Serial.printf(
        "LINE state=%s ready=%d line=%d correction=%d"
        " | raw L/M/R:",
        calibrating_
            ? "calibrating"
            : (ready_ ? "tracking" : "not-ready"),
        ready_,
        detected_,
        correction_
    );

    for (uint8_t i = 0; i < Count; ++i) {
        Serial.printf(" %u", raw_[i]);
    }

    Serial.print(" | white L/M/R:");
    for (uint8_t i = 0; i < Count; ++i) {
        Serial.printf(" %u", whiteStrength_[i]);
    }

    Serial.print(" | range L/M/R:");
    for (uint8_t i = 0; i < Count; ++i) {
        Serial.printf(" %u-%u", minimum_[i], maximum_[i]);
    }

    Serial.println();
}
