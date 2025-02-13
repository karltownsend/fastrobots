#include "car.h"

Car::Car(Motor &motorL, Motor &motorR)
{
  this->motorL = motorL;
  this->motorR = motorR;
  deadzone = 0;
}

void Car::begin()
{
  motorL.begin();
  motorR.begin();
}

void Car::forward(byte pwm)  // pwm value in range 0..255
{
  motorL.forward(pwm);
  motorR.forward(pwm);
}

void Car::reverse(byte pwm)  // pwm value in range 0..255
{
  motorL.reverse(pwm);
  motorR.reverse(pwm);
}

void Car::stop()
{
  motorL.brake();
  motorR.brake();
  delay(1000);
  motorL.coast();
  motorR.coast();
}

void Car::setDeadzone(byte deadzone)
{
  this->deadzone = deadzone;
}
