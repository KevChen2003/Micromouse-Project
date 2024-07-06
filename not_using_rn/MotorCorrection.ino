#include <Arduino.h>
#include "motor.hpp"
#include "PIDController.hpp"
#include "dualEncoder.hpp"


#define LEFT_MOTOR_PWM 9
#define LEFT_MOTOR_DIR 10
#define RIGHT_MOTOR_PWM 11
#define RIGHT_MOTOR_DIR 12
#define LEFT_ENCODER1_PIN 2
#define LEFT_ENCODER2_PIN 7
#define RIGHT_ENCODER1_PIN 3
#define RIGHT_ENCODER2_PIN 8


float kp1 = 1.0, ki1 = 0.1, kd1 = 0.05; 
float kp2 = 1.2, ki2 = 0.1, kd2 = 0.06; 


mtrn3100::PIDController pid1(kp1, ki1, kd1);
mtrn3100::PIDController pid2(kp2, ki2, kd2);


mtrn3100::Motor leftMotor(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR);
mtrn3100::Motor rightMotor(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR);
mtrn3100::DualEncoder dualEncoder(LEFT_ENCODER1_PIN, LEFT_ENCODER2_PIN, RIGHT_ENCODER1_PIN, RIGHT_ENCODER2_PIN);

void setup() {
    Serial.begin(9600);
    dualEncoder.zeroAndSetTarget(0, 0); 
}

void loop() {
    dualEncoder.readLeftEncoder();
    dualEncoder.readRightEncoder();
    float leftRotation = dualEncoder.getLeftRotation();
    float rightRotation = dualEncoder.getRightRotation();
    float pwmInput = 100;  // Example PWM input (adjust as needed)
    float leftPWM = pwmInput + pid1.compute(leftRotation);
    float rightPWM = pwmInput + pid2.compute(rightRotation);

    leftMotor.setPWM(leftPWM);
    rightMotor.setPWM(-rightPWM);


    Serial.print("Left Rotation: ");
    Serial.println(leftRotation);
    Serial.print("Right Rotation: ");
    Serial.println(rightRotation);

    delay(100);  // Adjust as needed for your control loop frequency
}
