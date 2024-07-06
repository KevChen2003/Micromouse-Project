
#include <Arduino.h>
#include "Robot.hpp"

// Define your pin configurations here
#define LEFT_MOTOR_PWM 9
#define LEFT_MOTOR_DIR 10
#define RIGHT_MOTOR_PWM 11
#define RIGHT_MOTOR_DIR 12
#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 7
#define RIGHT_ENCODER_A 3
#define RIGHT_ENCODER_B 8
#define IMU_ADDRESS 0x69  // Replace with your IMU's address

const float WHEEL_RADIUS = 16;
const float WHEEL_BASE = 90;

mtrn3100::Encoder left_encoder(LEFT_ENCODER_A,LEFT_ENCODER_B);
mtrn3100::Encoder right_encoder(RIGHT_ENCODER_A,RIGHT_ENCODER_B);


mtrn3100::Robot robot(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR, RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR,
                      LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B,
                      WHEEL_RADIUS, WHEEL_BASE, IMU_ADDRESS);

void setup() {
    Serial.begin(115200);
}

void loop() {


    float left_rotation = left_encoder.getRotation();
    float right_rotation = right_encoder.getRotation();

    Serial.print("left rotation");
    Serial.println(left_rotation);
    
    Serial.print("right rotation");
    Serial.println(right_rotation);
    delay(1000);
}

