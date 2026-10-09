#include "../include/motors.hpp"

// Constructor
MotorController::MotorController(int in1, int in2, int in3, int in4) {
    _in1 = in1;
    _in2 = in2;
    _in3 = in3;
    _in4 = in4;
}

// Set pin modes
void MotorController::init() {
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_in3, OUTPUT);
    pinMode(_in4, OUTPUT);
    stop();
}

// THIS IS THE MAIN MOTOR CONTROL LOGIC
// this works by taking a speed value (-255 to 255) and setting the motor direction (+/-)and PWM (w/ ADC) accordingly
// all we have to do is make functions that will call this function specifically for different directions/speeds
void MotorController::setMotor(int pinA, int pinB, int speed) { //speed is any integer -255 to 255
                                                                // positive for forward, negative for reverse, 0 to stop
    if (speed > 0) {
        analogWrite(pinA, speed);   // Speed via PWM
        analogWrite(pinB, 0);       // Keep the inactive input low through PWM
    } else if (speed < 0) {
        analogWrite(pinA, 0);
        analogWrite(pinB, -speed);  // Convert negative to positive for PWM
    } else {
        analogWrite(pinA, 0);       // Stop/coast with both inputs low
        analogWrite(pinB, 0);
    }
}

/*void MotorController::setMotor(int pinA, int pinB, int speed) {
    if (speed > 0) {
        ledcWrite(pinA, speed);   // ESP32 PWM speed
        digitalWrite(pinB, LOW);    // Ground for direction
    } else if (speed < 0) {
        digitalWrite(pinA, LOW);
        ledcWrite(pinB, -speed);  // ESP32 PWM speed (inverted)
    } else {
        digitalWrite(pinA, LOW);    // Stop/Coast
        digitalWrite(pinB, LOW);
    }
}
 */

// Basic Directional Controls
void MotorController::forward(int speed) {
    setMotor(_in1, _in2, speed);
    setMotor(_in3, _in4, speed);
}

void MotorController::backward(int speed) {
    setMotor(_in1, _in2, -speed);
    setMotor(_in3, _in4, -speed);
}

void MotorController::turnLeft(int speed) {
    setMotor(_in1, _in2, -speed); // Left wheel reverse
    setMotor(_in3, _in4, speed);  // Right wheel forward
}

void MotorController::turnRight(int speed) {
    setMotor(_in1, _in2, speed);   // Left wheel forward
    setMotor(_in3, _in4, -speed);  // Right wheel reverse
}

void MotorController::stop() {
    setMotor(_in1, _in2, 0);
    setMotor(_in3, _in4, 0);
}

// Joystick Control (Differential Steering)
void MotorController::driveJoystick(int x, int y) {
    int leftSpeed = constrain(y + x, -255, 255);
    int rightSpeed = constrain(y - x, -255, 255);

    setMotor(_in1, _in2, leftSpeed);
    setMotor(_in3, _in4, rightSpeed);
}