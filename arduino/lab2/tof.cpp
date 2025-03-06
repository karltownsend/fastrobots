#include "tof.h"
#include "constants.h"

uint16_t Tof::getRangeReal()
{
  while (!checkForDataReady())
    {
      delay(1);
    }
  
  time_now = millis();  
  distance = getDistance();
  clearInterrupt();
  return distance;
}

uint16_t Tof::getRangeExtrapolation()
{
  time_now = millis();
  if (checkForDataReady()) {
    distance = getDistance();
    clearInterrupt();

    // start new ranging
    //startRanging();

    // update previous distance and time
    dist_neg2 = dist_neg1;
    dist_neg1 = distance;
    time_neg2 = time_neg1;
    time_neg1 = time_now;

  } else {
    // do an extrapolation based on last 2 ToF values
    slope = (dist_neg2 - dist_neg1) / (time_neg2 - time_neg1);
    distance = dist_neg1 - slope * (time_neg1 - time_now);
  }

  return distance;
}

void Tof::stop()
{
  clearInterrupt();
  stopRanging();
}

void Tof::setDist1(float dist)
{
  dist_neg1 = dist;
}
void Tof::setDist2(float dist)
{
  dist_neg2 = dist;
}
void Tof::setTime1(float time)
{
  time_neg1 = time;
}
void Tof::setTime2(float time)
{
  time_neg2 = time;
}
