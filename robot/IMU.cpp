#include "IMU.hpp"
#include <Wire.h>
#include <Arduino.h>

void IMU::setupIMU(MPU6050& mpu) {
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status != 0) { } // Stops everything if it could not connect to the MPU6050
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true, true); // Calculates offsets for gyroscope and accelerometer
    Serial.println("Done!\n");
}

void IMU::updateIMU(MPU6050& mpu, float* yawReadings, int numReadings, int& index, unsigned long& timer, Adafruit_SSD1306& display) {
    mpu.update();
    if (millis() - timer > 1000) { // Prints data every second
        Serial.print(F("ACCELERO  X: "));
        Serial.print(mpu.getAccX());
        Serial.print("\tY: ");
        Serial.print(mpu.getAccY());
        Serial.print("\tZ: ");
        Serial.println(mpu.getAccZ());
    
        Serial.print(F("GYRO      X: "));
        Serial.print(mpu.getGyroX());
        Serial.print("\tY: ");
        Serial.print(mpu.getGyroY());
        Serial.print("\tZ: ");
        Serial.println(mpu.getGyroZ());
    
        Serial.print(F("ACC ANGLE X: "));
        Serial.print(mpu.getAccAngleX());
        Serial.print("\tY: ");
        Serial.println(mpu.getAccAngleY());
        
        Serial.print(F("ANGLE     X: "));
        Serial.print(mpu.getAngleX());
        Serial.print("\tY: ");
        Serial.print(mpu.getAngleY());
        Serial.print("\tZ: ");
        Serial.println(mpu.getAngleZ());

        float currentYaw = mpu.getAngleZ(); // Gets current yaw reading

        // Updates the array with new yaw reading
        yawReadings[index] = currentYaw;
        index = (index + 1) % numReadings; // Increments index and wrap around if needed

        // Calculates the average of the last numReadings yaw readings
        float yawAverage = 0;
        for (int i = 0; i < numReadings; i++) {
            yawAverage += yawReadings[i];
        }
        yawAverage /= numReadings;

        // Prints moving yaw average to serial monitor
        Serial.print(F("Yaw (Moving Average): "));
        Serial.println(yawAverage);
        Serial.println(F("=====================================================\n"));
        timer = millis();

        // Updates the OLED with the moving yaw average
        display.clearDisplay();
        display.setTextSize(1); // Normal 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE); // White text
        display.setCursor(0, 0); // Starts at the top-left corner
        display.print(F("Yaw (Moving Average): ")); // Since we are reusing this text, it is better to allocate it to flash memory instead of SRAM
        display.println(yawAverage);
        display.display();
    }
}