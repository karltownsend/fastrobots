#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID
{
private:
  float *input, *output, *setpoint;
  float kp, ki, kd;
  float Pterm, Iterm, Dterm;
  float error, delta;
  float errSum = 0.0;
  float prev_error = 0.0;
  unsigned long time_now, dt;
  unsigned long time_prev = millis();

public:
  PID(float* input, float* output, float* setpoint,
      float kp, float ki, float kd) : 
      input(input), output(output), setpoint(setpoint),
      kp(kp), ki(ki), kd(kd) {};
    
  void begin();
  void compute();
  void setConstants(float kp, float ki, float kd);
  void getPID(float* P, float* I, float* D);
};

#endif