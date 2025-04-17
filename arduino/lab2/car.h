#ifndef CAR_H
#define CAR_H

#include <Arduino.h>
#include "motor.h"
#include "constants.h"

class Car
{
private:
  byte pwm_max = MAX;
  byte deadzone = 40;
  int motor_pwm;
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
  void coast();
  void stop();

  void setPwmMax(byte pwm_max);
  void setDeadzone(byte deadzone);
  void setCorrection(float correction);
  int setLinearSpeed(float pid_out);
  int setRotateSpeed(float pid_out);
};

#endif