#include "tof.h"
#include "constants.h"

float Tof::getRangeReal() {
  while (!checkForDataReady())
    delay(1);

  getRealValues();
  return distance;
}

float Tof::getRangeExtrapolation() {
  if (checkForDataReady()) {
    getRealValues();

  } else {
    if (dist_neg2 == -1 or dist_neg1 == -1) {
      // return a real distance fro the TOF sensor
      distance = getRangeReal();
    } else {
      // return an extrapolation based on last 2 ToF values
      distance = dist_neg2 + slope * (millis() - time_neg2);
      distance = max(distance, MIN);
    }
  }
  return distance;
}

void Tof::getRealValues() {
  distance = getDistance();
  clearInterrupt();

  // update previous distance and time
  time_neg2 = time_neg1;
  dist_neg2 = dist_neg1;

  time_neg1 = millis();
  dist_neg1 = distance;

  // calculate slope
  delta_dist = dist_neg2 - dist_neg1;
  delta_time = time_neg2 - time_neg1;
  slope = delta_dist / delta_time;
}

void Tof::stop() {
  clearInterrupt();
  stopRanging();
  dist_neg1 = -1;
  dist_neg2 = -1;
}