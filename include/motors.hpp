#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <Arduino.h>
#include "sdkconfig.h"

class MotorController {
  public:
    // Constructor: Takes the 4 control pins connected to DRV8833
    MotorController(int in1, int in2, int in3, int in4);

    // Call inside setup() to set pins to OUTPUT mode
    void init();

    // Direct movement methods
    void forward(int speed);
    void backward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void stop();

    // Joystick control: Takes X and Y coordinates (-255 to 255)
    void driveJoystick(int x, int y);

  private:
    int _in1, _in2, _in3, _in4;

    // Helper method to set direction and PWM speed for a single motor
    void setMotor(int pinA, int pinB, int speed);
};

#endif