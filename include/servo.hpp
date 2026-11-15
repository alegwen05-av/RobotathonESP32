#ifndef SERVO_HPP
#define SERVO_HPP

#include <stdint.h>
#include <ESP32Servo.h>

class ServoController {
  public:
    bool begin(int pin);
    void advancePosition();

  private:
    Servo servo_;
    uint8_t positionIndex_ = 0;
};

#endif