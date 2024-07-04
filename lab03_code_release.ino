#include <Wire.h>
#include "Robot.hpp"


#define LEFT_MOTOR_PWM 9
#define LEFT_MOTOR_DIR 10
#define RIGHT_MOTOR_PWM 11
#define RIGHT_MOTOR_DIR 12

#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 7
#define RIGHT_ENCODER_A 3
#define RIGHT_ENCODER_B 8

#define LIDAR_FRONT_PIN A3 


const float WHEEL_RADIUS = 16; 
const float WHEEL_BASE = 90;  


mtrn3100::Robot robot(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR, RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR,
                      LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B,
                      WHEEL_RADIUS, WHEEL_BASE);

void setup() {
    Serial.begin(9600);
}





void loop() {
    const char* commands = "frfl";
    robot.setCurrentPWM(200); 
    robot.chainMovements(commands);
    while (true) {
        robot.stopMotors();
        delay(1000);
    }
}
