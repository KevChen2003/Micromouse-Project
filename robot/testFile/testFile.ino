#include "Wire.h"
#include "motor.hpp"
#include "dualEncoder.hpp"
#include "encoderOdometry.hpp"

#define EN_1_A 2 // Pin A for DC motor encoder 1
#define EN_1_B 7 // Pin B for DC motor encoder 1
#define EN_2_A 3 // Pin A for DC motor encoder 2
#define EN_2_B 8 // Pin B for DC motor encoder 2

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(16,90); // Wheel radius & axial length
mtrn3100::Motor L_Motor(9, 10); // correct
mtrn3100::Motor R_Motor(11, 12);

void setup() {
    // Initialization code for components
    Serial.begin(9600);

    Serial.println('hello');

    Wire.begin();

    L_Motor.setPWM(255); 
    R_Motor.setPWM(-255); //
}

void loop() {
    delay(50);
    // Update encoder odometry
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());

    // Print position information (optional)

    Serial.print("X : ");
    Serial.println(encoder_odometry.getX());
    Serial.print("Y : ");
    Serial.println(encoder_odometry.getY());
    Serial.print("H : ");
    Serial.println(encoder_odometry.getH());

    // Add a delay to control loop rate
    delay(15);
}