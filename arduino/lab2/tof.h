#ifndef TOF_H
#define TOF_H

#include <Arduino.h>
#include <SparkFun_VL53L1X.h>

class Tof : public SFEVL53L1X
{
private:
  float dist_neg1 = -1;
  float dist_neg2 = -1;
  float slope, delta_dist;
  unsigned long time_neg1, time_neg2;
  long delta_time;

public:
  Tof() {};  // Default constructor
  Tof(byte shutdown) : SFEVL53L1X(Wire, shutdown, -1) {}

  float distance;
  float getRangeReal();
  float getRangeExtrapolation();
  void getRealValues();
  void stop();

  /* void setDist1(float dist);
  void setDist2(float dist);
  void setTime1(unsigned long time);
  void setTime2(unsigned long time);*/
};

#endif