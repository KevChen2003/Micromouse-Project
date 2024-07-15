#pragma once

#include <VL6180X.h>
#include <Arduino.h>
#include <Wire.h>

namespace mtrn3100 {
class Lidar {
  public:
    void setupLidars(int lidar1_pin, int lidar2_pin, int lidar3_pin);
    void updateLidars();

    float getLeftLidar();
    float getRightLidar();
    float getFrontLidar();

    private:
    VL6180X lidar1, lidar2, lidar3; // Creates lidar 1, lidar 2 & lidar 3

};
}