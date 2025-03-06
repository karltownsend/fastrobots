#include "car.h"
#include "motor.h"

Car::Car() : 
    motorL(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2),
    motorR(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2) {}

void Car::begin() {
  motorL.begin();
  motorR.begin();
}

// Apply a correction factor if the car does not run straight.
// correction is applied to the left wheel and is in range 0..2
// where 1 indicates no correction
void Car::forward(byte pwm)  // pwm value in range 0..255
{
  motorL.forward(pwm * correction);
  motorR.forward(pwm);
}

// Apply a correction factor if the car does not run straight.
// correction is applied to the left wheel and is in range 0..2
// where 1 indicates no correction
void Car::reverse(byte pwm)  // pwm value in range 0..255
{
  motorL.reverse(pwm * correction);
  motorR.reverse(pwm);
}

void Car::rotateLeft(byte pwm)  // pwm value in range 0..255
{
  motorL.reverse(pwm * correction);
  motorR.forward(pwm);
}

void Car::rotateRight(byte pwm)  // pwm value in range 0..255
{
  motorL.forward(pwm * correction);
  motorR.reverse(pwm);
}

void Car::stop() {
  motorL.brake();
  motorR.brake();
  delay(1000);
  motorL.coast();
  motorR.coast();
}

void Car::setDeadzone(byte deadzone) {
  this->deadzone = deadzone;
}

void Car::setCorrection(float correction) {
  this->correction = correction;
  Serial.print("correction: ");
  Serial.println(correction);
}

float Car::setLinearSpeed(float pid_output)
// update pwm value of motors based on pid_out
// pid_out is in range -100..100
// positive numbers mean forward, negative numbers mean reverse
//
// Use deadzone to set the minimum pwm value to get the car to move,
// then scale the remaining values
{
  float motor_pwm = deadzone + abs(pid_output) * (MAX - deadzone) / 100;
  if (abs(pid_output) < 0.1) {
    motor_pwm = 0;
    stop();
  } else if (pid_output >= 0.1) {
    forward(motor_pwm);
  } else {
    reverse(motor_pwm);
    motor_pwm = -motor_pwm;  // return a negative number for logging
  }
  return motor_pwm;
}

float Car::setRotateSpeed(float pid_output)
// update pwm value of motors based on pid_out
// pid_out is in range -100..100
// positive numbers mean rotate left, negative numbers mean rotate right
//
// Use deadzone to set the minimum pwm value to get the car to move,
// then scale the remaining values
{
  float motor_pwm = deadzone + abs(pid_output) * (MAX - deadzone) / 100;
  if (abs(pid_output) < 0.1) {
    motor_pwm = 0;
    stop();
  } else if (pid_output >= 0.1) {
    rotateLeft(motor_pwm);
  } else {
    rotateRight(motor_pwm);
    motor_pwm = -motor_pwm;  // return a negative number for logging
  }
  return motor_pwm;
}