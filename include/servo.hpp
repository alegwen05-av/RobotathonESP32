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
    bool raised_ = false;
    bool moving_ = false;
    unsigned long movementStartedMs_ = 0;
};

#endif