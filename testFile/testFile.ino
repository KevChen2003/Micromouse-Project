// OLED not done
// PID controller not done
// implement the simple movement functions
// hardware: IMU broken, needs checking

#include "robot.hpp"


int count = 0;

void setup() {
    // Initialization code for components
    Wire.begin();
    Serial.begin(9600);

    lidar.setupLidars(leftLidar_pin, rightLidar_pin, frontLidar_pin);
    imu.setupIMU(mpu);
    // oled.setupOLED(display);

    // l_forward_pid.zeroAndSetTarget(encoder.getLeftRotation(), target);    
    // r_forward_pid.zeroAndSetTarget(-encoder.getRightRotation(), target);
}

void loop() {
    delay(50);
    // Update encoder odometry
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());

    move(5);


    // moveForward(1);
    
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

    // Add a delay to control loop rate
    delay(1500);
};

bool move(int nCells) {
    while (count < nCells) {
        l_forward_pid.zeroAndSetTarget(encoder.getLeftRotation(), 250/16);    
        r_forward_pid.zeroAndSetTarget(-encoder.getRightRotation(), 250/16);
        while(!moveOneCellForward()){};
        count++;
        Serial.print("Count: ");
        Serial.println(count);
    }

    delay(1500);
    return false;
}

bool moveOneCellForward() {
    float pidL_signal = l_forward_pid.compute(encoder.getLeftRotation());
    float pidR_signal = r_forward_pid.compute(-encoder.getRightRotation());

    Serial.print("Left pid: ");
    Serial.println(pidL_signal);

    Serial.print("Right pid: ");
    Serial.println(pidR_signal);

    L_Motor.setPWM(pidL_signal);
    R_Motor.setPWM(-pidR_signal);

    if (abs(pidL_signal) < 12 && abs(pidR_signal < 12)) {
        L_Motor.setPWM(0);
        R_Motor.setPWM(0);
        Serial.println("Stopped");
        delay(1500);
        return true;
    }

    delay(1500);
    return false;
}