#ifndef CAR_H
#define CAR_H

#include <Arduino.h>
#include "motor.h"
#include "constants.h"

class Car
{
private:
  byte deadzone = 40;
  float correction = 1.1;

public:
  Car(); // Default Constructor
  Car(Motor &motorL, Motor &motorR) : motorL(motorL), motorR(motorR) {};

  Motor motorL;
  Motor motorR;

  void begin();
  void forward(byte pwm);  // pwm value in range 0..255
  void reverse(byte pwm);  // pwm value in range 0..255
  void rotateLeft(byte pwm);
  void rotateRight(byte pwm);  
  void stop();

  void setDeadzone(byte deadzone);
  void setCorrection(float correction);
  float setLinearSpeed(float pid_out);
  float setRotateSpeed(float pid_out);
};

#endif