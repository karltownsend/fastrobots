#include "pid.h"

void PID::begin()
{
  errSum = 0.0;
  prev_error = 0.0;
}

void PID::compute()
{
  time_now = millis();
  dt = time_now - time_prev;
  time_prev = time_now;

  error = *input - *setpoint;
  errSum += error * dt;
  delta = (error - prev_error) / dt;

  prev_error = error;

  // correct for large integral errors
  if (abs(error) < 0.1)
    errSum = 0;
  
  // correct for wind-up
  errSum = constrain(errSum, -200, 200);
    
  // reset errSum if error is greater than 2 feet or 610 mm
  if (abs(error) > 610)
    errSum = 0;

  // calculate output and then clamp to +/- 100
  Pterm = kp * error;
  Iterm = ki * errSum;
  Dterm = kd * delta;

  *output = Pterm + Iterm + Dterm;
  *output = constrain(*output, -100, 100);
}

void PID::setConstants(float kp, float ki, float kd)
{
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;  
}

void PID::getPID(float* P, float* I, float* D)
{
  *P = Pterm;
  *I = Iterm;
  *D = Dterm;
}