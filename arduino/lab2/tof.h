#ifndef TOF_H
#define TOF_H

#include <Arduino.h>
#include <SparkFun_VL53L1X.h>

class Tof : public SFEVL53L1X
{
private:
  float dist_neg1, dist_neg2, slope;
  float time_neg1, time_neg2, time_now;

public:
  Tof() {};  // Default constructor
  Tof(char shutdown) : SFEVL53L1X(Wire, shutdown, -1) {}

  uint16_t distance;
  uint16_t getRangeReal();
  uint16_t getRangeExtrapolation();
  void stop();

  void setDist1(float dist);
  void setDist2(float dist);
  void setTime1(float time);
  void setTime2(float time);
};

#endif