// OLED not done
// PID controller not done
// implement the simple movement functions
// hardware: IMU broken, needs checking

#include "robot.hpp"


int count = 0;
float target = (250 / 16);

void setup() {
    // Initialization code for components
    Wire.begin();
    Serial.begin(9600);

    // lidar.setupLidars(leftLidar_pin, rightLidar_pin, frontLidar_pin);
    // imu.setupIMU(mpu);
    // oled.setupOLED(display);

    // l_forward_pid.zeroAndSetTarget(encoder.getLeftRotation(), target);    
    // r_forward_pid.zeroAndSetTarget(-encoder.getRightRotation(), target);
}

void loop() {
    delay(50);
    // Update encoder odometry
    // encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());

    // L_Motor.setPWM(255);
    // R_Motor.setPWM(-50);

    // encoder.readLeftEncoder();

    // Serial.print("left encoder: ");
    // Serial.println(encoder.getLeftRotation());

    // Serial.print("right encoder: ");
    // Serial.println(encoder.getRightRotation());

    // float pidL_signal = l_forward_pid.compute(encoder.getLeftRotation());
    // float pidR_signal = r_forward_pid.compute(-encoder.getRightRotation());

    // Serial.print("left pid: ");
    // Serial.println(pidL_signal);

    // Serial.print("right pid: ");
    // Serial.println(pidR_signal);

    // L_Motor.setPWM(pidL_signal);
    // R_Motor.setPWM(-pidR_signal);

    while (count < 5) {
        l_forward_pid.zeroAndSetTarget(encoder.getLeftRotation(), target);    
        r_forward_pid.zeroAndSetTarget(-encoder.getRightRotation(), target);
        while(!move()){};
        count++;
        Serial.print("Count: ");
        Serial.println(count);
    }


    // moveForward(1);
    
    // imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

    // Add a delay to control loop rate
    delay(1500);
};

bool move() {
    float pidL_signal = l_forward_pid.compute(encoder.getLeftRotation());
    float pidR_signal = r_forward_pid.compute(-encoder.getRightRotation());

    Serial.print("Left pid: ");
    Serial.println(pidL_signal);

    Serial.print("Right pid: ");
    Serial.println(pidR_signal);

    L_Motor.setPWM(pidL_signal);
    R_Motor.setPWM(-(pidR_signal));
    if (abs(pidL_signal) < 20 && abs(pidR_signal < 20)) {
        L_Motor.setPWM(0);
        R_Motor.setPWM(0);
        Serial.println("Stopped");
        return true;
    }

    return false;
}