#pragma once

#include <Adafruit_SSD1306.h>
#include <Arduino.h>

#define SCREEN_ADDRESS 0x3D // See OLED datasheet for address (0x3D for 128x64, 0x3C for 128x32)

namespace OLED {
  void setupOLED(Adafruit_SSD1306& display);
}