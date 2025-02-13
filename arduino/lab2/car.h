#ifndef CAR_H
#define CAR_H

#include <Arduino.h>
#include "motor.h"

class Car
{
private:
  Motor motorL;
  Motor motorR;
  byte deadzone;

public:
  Car() {}  // Default Constructor - do not use
  Car(Motor &motorL, Motor &motorR);

  void begin();
  void forward(byte pwm);  // pwm value in range 0..255
  void reverse(byte pwm);  // pwm value in range 0..255
  void stop();

  void setDeadzone(byte deadzone);
};

#endif