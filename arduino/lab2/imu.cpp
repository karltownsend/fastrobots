#include "imu.h"
#include "constants.h"

void Imu::begin()
{
  Wire.begin();
  Wire.setClock(400000);

  ICM_20948_I2C::begin(Wire, AD0_VAL);

  Serial.print(F("Initialization of the IMU returned: "));
  Serial.println(statusString());
  while (status != ICM_20948_Stat_Ok) {
    Serial.println("Trying again...");
    delay(500);
  }
  Serial.println("IMU Sensor online!");

  ICM_20948_fss_t myFSS;
  myFSS.a = gpm2;
  myFSS.g = dps1000;

  setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (status != ICM_20948_Stat_Ok) {
    Serial.print(F("setFullScale returned: "));
    Serial.println(statusString());
  }
}