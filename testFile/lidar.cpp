#include "Lidar.hpp"

void mtrn3100::Lidar::setupLidars(int lidar1_pin, int lidar2_pin, int lidar3_pin) {
    pinMode(lidar1_pin, OUTPUT); // Initialises lidar1_pin as an output 
    pinMode(lidar2_pin, OUTPUT); // Initialises lidar2_pin as an output 
    pinMode(lidar3_pin, OUTPUT); // Initialises lidar3_pin as an output 
    digitalWrite(lidar1_pin, LOW); // Initially disables lidar1
    digitalWrite(lidar2_pin, LOW); // Initially disables lidar2
    digitalWrite(lidar3_pin, LOW); // Initially disables lidar3

    // Serial.print("before 1");
    delay(50);
    // Avoids address conflict between lidars
    digitalWrite(lidar1_pin, HIGH); // Enables lidar1 first
    delay(50);
    lidar1.init();
    lidar1.configureDefault();
    lidar1.setScaling(1);
    lidar1.setTimeout(250);
    lidar1.setAddress(0x52); // Changes the address of lidar1 (from 0x29)

    delay(50);
    // Serial.print(before 2");

    digitalWrite(lidar2_pin, HIGH); // Enables lidar2 second
    delay(50);
    lidar2.init();
    lidar2.configureDefault();
    lidar2.setTimeout(250);
    lidar2.setAddress(0x54); // Changes the address of lidar2 (from 0x29)

    delay(50);
    // Serial.print("before 3");

    digitalWrite(lidar3_pin, HIGH); // Enables lidar3 third
    delay(50);
    lidar3.init();
    lidar3.configureDefault();
    lidar3.setTimeout(250);
    lidar3.setAddress(0x56); // Changes the address of lidar2 (from 0x29)
}

void mtrn3100::Lidar::updateLidars() {
    Serial.print("Distance: ");
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

float mtrn3100::Lidar::getLeftLidar() {
    return lidar1.readRangeSingleMillimeters();
}

float mtrn3100::Lidar::getRightLidar() {
    return lidar2.readRangeSingleMillimeters();
}

float mtrn3100::Lidar::getFrontLidar() {
    return lidar3.readRangeSingleMillimeters();
}