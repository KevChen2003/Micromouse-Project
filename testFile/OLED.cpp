#include "OLED.hpp"
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Arduino.h>

void mtrn3100::OLED::setupOLED(Adafruit_SSD1306& display) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;) ; // Don't proceed, loop forever
    }
    display.display(); // Adafruit splash screen buffer content
    delay(2000);
    display.clearDisplay(); // Clears the buffer
}