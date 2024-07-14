// OLED not done
// PID controller not done
// implement the simple movement functions
// hardware: IMU broken, needs checking

#include "robot.hpp"

void setup() {
    // Initialization code for components
    Wire.begin();
    Serial.begin(9600);

    lidar.setupLidars(lidar1_pin, lidar2_pin, lidar3_pin);
    imu.setupIMU(mpu);
    oled.setupOLED(display);
}

void loop() {
    delay(50);
    // Update encoder odometry
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());

    moveForward(2);
    
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer, display);

    // Add a delay to control loop rate
    delay(1500);
};

void stopMotors(){
  L_Motor.setPWM(0); 
  R_Motor.setPWM(0);
}