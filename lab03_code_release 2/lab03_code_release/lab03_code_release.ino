#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "BangBangController.hpp"

#define MOT1PWM 9 // PIN 9 is a PWM pin
#define MOT1DIR 10

#define MOT2PWM 11
#define MOT2DIR 12

mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

#define EN_A 2 // PIN 2 is an interupt
#define EN_B 4
mtrn3100::Encoder encoder(EN_A, EN_B);

mtrn3100::BangBangController controller(120,0);


void setup() {
  Serial.begin(9600);
  controller.zeroAndSetTarget(encoder.getRotation(), 2.0); // Set the target as 2 Radians
}

void loop() {
    // Rotate one revolution in one direction

    // write motor move functions that will: either rotate or move the robot forward
    // for moving forward, you need to calculate how many rotations from the encoder is required for moving the robot 1 cell 
    // for rotation, you need to calculate how many rotations from the encoder is required for a 90 / 180 / 270 degree turn
    // each cell is 250mm by 250mm
    // wheel radius is 16mm

    // motor move forward: input a certain number of cells you want the motor to move forward, then calculate distance and how many rotations
    // from the encoder you need

    // motor rotate: input number of degrees or some form left / right turning, then calculate how many rotations each wheel needs to move
    // to turn 90 degrees in either direction

    // connect encoder to other motor

    // get PID working with encoder
    
    // forward: 1 (right) -> 50, 2 (left) -> -70

    motor1.setPWM(0); // Full speed forward
    motor2.setPWM(0);

    // only works on left rotation
    float rotation_number = encoder.getRotation();
    Serial.println(rotation_number);

    // controller.compute()
    //     float currentPosition = encoder.getRotation();
    
    // // Compute control signal using BangBangController
    // float controlSignal = controller.compute(currentPosition);
    
    // // Apply control signal to your system (e.g., adjust motor1 speed)
    // motor1.setPWM(controlSignal);
    
    // // Optional: Print out the error for debugging
    // Serial.print("Error: ");
    // Serial.println(controller.getError());

    // // Optional: Add a delay to control loop frequency
    // delay(1000); // Adjust delay as needed
    
    
    
    
}
