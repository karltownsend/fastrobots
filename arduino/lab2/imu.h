#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <ICM_20948.h>

class Imu : public ICM_20948_I2C
{
public:
  Imu() {};  // Default constructor

  void begin();
};

#endif