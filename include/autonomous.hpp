#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP

#include <stdint.h>
#include "colorSensor.hpp"

class MotorController;

class AutonomousController {
  public:
    AutonomousController(ColorSensor& colorSensor, MotorController& motors);
    void begin();
    void update();

  private:
    enum class State {
        LearningStartColor,
        LeavingStartTile,
        SeekingStartColor,
        Stopped
    };

    ColorSensor& colorSensor_;
    MotorController& motors_;
    State state_ = State::LearningStartColor;
    DetectedColor startColor_ = DetectedColor::None;
    DetectedColor candidateColor_ = DetectedColor::None;
    uint8_t candidateReadCount_ = 0;
    unsigned long lastReadingMs_ = 0;
};

#endif
