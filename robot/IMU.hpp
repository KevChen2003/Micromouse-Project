#pragma once

#include <MPU6050_light.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

namespace IMU {
    void setupIMU(MPU6050& mpu);
    void updateIMU(MPU6050& mpu, float* yawReadings, int numReadings, int& index, unsigned long& timer, Adafruit_SSD1306& display);
}