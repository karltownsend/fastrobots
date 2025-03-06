#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor
{
private:
  byte pin1;
  byte pin2;

public:
  Motor(byte pin1, byte pin2) : pin1(pin1), pin2(pin2) {}

  void begin();
  void forward();
  void forward(byte pwm);  // pwm value in range 0..255
  void reverse();
  void reverse(byte pwm);  // pwm value in range 0..255
  void coast();
  void brake();
  void setRaw(byte pwm1, byte pwm2);
};

#endif