#ifndef SERVO_HPP
#define SERVO_HPP

#include <ESP32Servo.h>

class ServoController {
  public:
    bool begin(int pin);
    void toggle();

  private:
    Servo servo_;
    bool atEndPosition_ = false;
};

#endif