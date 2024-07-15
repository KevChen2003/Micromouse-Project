#pragma once

#include <VL6180X.h>
#include <Arduino.h>

namespace Lidar {
    void setupLidars(VL6180X& lidar1, VL6180X& lidar2, VL6180X& lidar3, int lidar1_pin, int lidar2_pin, int lidar3_pin);
    void updateLidars(VL6180X& lidar1, VL6180X& lidar2, VL6180X& lidar3);
}