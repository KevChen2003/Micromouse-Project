// OLED not done
// PID controller not done
// implement the simple movement functions
// hardware: IMU broken, needs checking

#include "robot.hpp"

void setup() {
    // Initialization code for components
    Wire.begin();
    Serial.begin(9600);

    lidar.setupLidars(leftLidar_pin, rightLidar_pin, frontLidar_pin);
    imu.setupIMU(mpu);
    // oled.setupOLED(display);
}

void loop() {
    delay(50);
    // Update encoder odometry
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());

    moveForward(1);
    
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

    // Add a delay to control loop rate
    delay(1500);
};
