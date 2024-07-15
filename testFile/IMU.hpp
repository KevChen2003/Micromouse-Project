#pragma once

#include <MPU6050_light.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>


namespace mtrn3100 {
  class IMU{
    public:
    void setupIMU(MPU6050& mpu);
    void updateIMU(MPU6050& mpu, float* yawReadings, int numReadings, int& index, unsigned long& timer);
};
}