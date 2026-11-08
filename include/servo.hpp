#ifndef SERVO_HPP
#define SERVO_HPP

#include <ESP32Servo.h>

class ServoController {
  public:
    bool begin(int pin);
    void toggle();
    void update();

  private:
    Servo servo_;
    bool moving_ = false;
    bool forwardNext_ = true;
    unsigned long phaseStartedMs_ = 0;
};

#endif