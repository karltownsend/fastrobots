#include "Motor.h"
#include "constants.h"

void Motor::begin()
{
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  coast();
}

void Motor::forward()
{
  analogWrite(pin1, MAX);
  analogWrite(pin2, MIN);
}

void Motor::forward(byte pwm)
{
  analogWrite(pin1, pwm);
  analogWrite(pin2, MIN);
}

void Motor::reverse()
{
  analogWrite(pin1, MIN);
  analogWrite(pin2, MAX);
}

void Motor::reverse(byte pwm)
{
  digitalWrite(pin1, MIN);
  analogWrite(pin2, pwm);
}

void Motor::coast()
{
  analogWrite(pin1, MIN);  
  analogWrite(pin2, MIN);
}

void Motor::brake()
{
  analogWrite(pin1, MAX);  
  analogWrite(pin2, MAX);
}

void Motor::setRaw(byte pwm1, byte pwm2)
{
  analogWrite(pin1, pwm1);  
  analogWrite(pin2, pwm2);
}