#include "Lidar.hpp"
#include <Wire.h>
#include <Arduino.h>

void Lidar::setupLidars(VL6180X& lidar1, VL6180X& lidar2, VL6180X& lidar3, int lidar1_pin, int lidar2_pin, int lidar3_pin) {
    pinMode(lidar1_pin, OUTPUT); // Initialises lidar1_pin as an output 
    pinMode(lidar2_pin, OUTPUT); // Initialises lidar2_pin as an output 
    pinMode(lidar3_pin, OUTPUT); // Initialises lidar3_pin as an output 
    digitalWrite(lidar1_pin, LOW); // Initially disables lidar1
    digitalWrite(lidar2_pin, LOW); // Initially disables lidar2
    digitalWrite(lidar3_pin, LOW); // Initially disables lidar3

    // Avoids address conflict between lidars
    digitalWrite(lidar1_pin, HIGH); // Enables lidar1 first
    delay(50);
    lidar1.init();
    lidar1.configureDefault();
    lidar1.setTimeout(250);
    lidar1.setAddress(0x54); // Changes the address of lidar1 (from 0x29)

    delay(50);

    digitalWrite(lidar2_pin, HIGH); // Enables lidar2 second
    delay(50);
    lidar2.init();
    lidar2.configureDefault();
    lidar2.setTimeout(250);
    lidar2.setAddress(0x56); // Changes the address of lidar2 (from 0x29)

    delay(50);

    digitalWrite(lidar3_pin, HIGH); // Enables lidar3 third
    delay(50);
    lidar3.init();
    lidar3.configureDefault();
    lidar3.setTimeout(250);
    lidar3.setAddress(0x58); // Changes the address of lidar2 (from 0x29)
}

void Lidar::updateLidars(VL6180X& lidar1, VL6180X& lidar2, VL6180X& lidar3) {
    Serial.print(lidar1.readRangeSingleMillimeters()); // Prints the range value of lidar1 to serial monitor
    Serial.print(" | ");
    Serial.print(lidar2.readRangeSingleMillimeters()); // Prints the range value of lidar2 to serial monitor
    Serial.print(" | ");
    Serial.print(lidar3.readRangeSingleMillimeters()); // Prints the range value of lidar3 to serial monitor
    Serial.println();
    if (lidar1.timeoutOccurred()) { 
        Serial.print("lidar 1 TIMEOUT");
    }
    if (lidar2.timeoutOccurred()) { 
        Serial.print("lidar 2 TIMEOUT"); 
    }
    if (lidar3.timeoutOccurred()) { 
        Serial.print("lidar 3 TIMEOUT"); 
    }
    delay(100); // Waits 0.1 second for next scan
}
