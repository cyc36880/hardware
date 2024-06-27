#include <Wire.h>
#include "LSM6DS3TRC.h"

LSM6DS3TRC lsm = LSM6DS3TRC();

void setup() {
  Serial.begin(9600);
  //I2C
//  lsm.LSM6DS3TRC_MODE = LSM6DS3TRC_MODE_I2C;

  //SPI
  lsm.Pin_CS=10;
  lsm.LSM6DS3TRC_MODE = LSM6DS3TRC_MODE_SPI;
  
  lsm.begin();

  Serial.println("LSM6DS3TRC sensor test");
  if (!lsm.begin()) {
    Serial.println("LSM6DS3TRC not found, check wiring?");
    while (1);
  }
  Serial.println("LSM6DS3TRC found!");
}

void loop() {
  uint8_t status;
  status = lsm.Get_Status();
  if (status & LSM6DS3TRC_STATUS_ACCELEROMETER)
  {
    float acc[3] = {0};
    lsm.Get_Acceleration(LSM6DS3TRC_ACC_FSXL_2G, acc);
    Serial.print("acc:X:");
    Serial.print(acc[0]);
    Serial.print("Y:");
    Serial.print(acc[1]);
    Serial.print("Z:");
    Serial.println(acc[2]);
  }
  if (status & LSM6DS3TRC_STATUS_GYROSCOPE)
  {
    float gyr[3] = {0};
    lsm.Get_Acceleration(LSM6DS3TRC_GYR_FSG_2000, gyr);
    Serial.print("gyr: X: ");
    Serial.print(gyr[0]);
    Serial.print("Y: ");
    Serial.print(gyr[1]);
    Serial.print("Z:");
    Serial.println(gyr[2]);
  }
  if (status & LSM6DS3TRC_STATUS_TEMPERATURE)
  {
    Serial.print("temp:");
    Serial.println(lsm.Get_Temperature());
  }
  delay(100);
}
