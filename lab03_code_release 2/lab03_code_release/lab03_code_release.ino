#include "Motor.hpp"
#include "PIDController.hpp"
#include "BangBangController.hpp"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"

#define MOT1PWM 9 // PIN 9 is a PWM pin
#define MOT1DIR 10

#define MOT2PWM 11
#define MOT2DIR 12

mtrn3100::Motor motorR(MOT1PWM,MOT1DIR);
mtrn3100::Motor motorL(MOT2PWM,MOT2DIR);

#define EN1_A 2 // PIN 2 is an interupt
#define EN1_B 7

#define EN2_A 3
#define EN2_B 8

// test pid values
const float kp = 200.0; 
const float ki = 25.0;
const float kd = 0.0;

// const int kp = 1;
// const int ki = 1;
// const int kd = 1;

mtrn3100::DualEncoder encoder(EN1_A, EN1_B,EN2_A, EN2_B);
mtrn3100::EncoderOdometry encoder_odometry(16,90); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH


mtrn3100::BangBangController controller(120,0);
mtrn3100::PIDController pidR(kp, ki, kd);
mtrn3100::PIDController pidL(kp, ki, kd);

// right encoder reads negative

// const float targetL = 1 * (250 / (32.0 * PI));
// const float targetR = -1 * (250 / (32.0 * PI));

// target in radians? 
const float targetL = 250 / 16.0;
const float targetR = -250 / 16.0;

void setup() {
  Serial.begin(9600);
  // controller.zeroAndSetTarget(encoderL.getRotation(), 2.0); // Set the target as 2 Radians

  // set target to 5 * cell
  pidL.zeroAndSetTarget(encoder.getLeftRotation(), targetL);
  pidR.zeroAndSetTarget(encoder.getRightRotation(), targetR);
}

bool running = true;

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

    // use IMU odometry and Encoder odometry?

    // get PID working with encoder
    
    // forward: 1 (right) -> 50, 2 (left) -> -70

    // motorR.setPWM(0); 
    // motorL.setPWM(0);
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
    
    float pidL_val = pidL.compute(encoder.getLeftRotation());
    float pidR_val = pidR.compute(encoder.getRightRotation());

    // might need to map pid value to pwm, but that requires the min and max of pid values

    int pmwL = pidL_val;
    int pmwR = pidR_val;

    if (abs(pidL_val - targetL) < 0.01 * targetL) {
      pmwL = 0;
    }

    if (abs(pidR_val - targetR) < 0.01 * targetL) {
      pmwR = 0;
    }

    motorL.setPWM(pmwL);
    motorR.setPWM(pmwR);

    // if (encoder.getLeftRotation() > targetL && encoder.getRightRotation() < targetR) {
    //   running = false;
    // }

    // while (running == false) {
    //   motorL.setPWM(0);
    //   motorR.setPWM(0);
    // }

    // PIDs at roughly: left 32800, right -32600

    Serial.print("pidL: ");
    Serial.println(pidL_val);

    Serial.print("pidR: ");
    Serial.println(pidR_val);

    float rotationL = encoder.getLeftRotation();
    Serial.print("Left Encoder: ");
    Serial.println(rotationL);

    // returning a negative value for positive rotation -> flip later
    float rotationR = encoder.getRightRotation();
    Serial.print("Right Encoder: ");
    Serial.println(rotationR);
    
    // // encoder odometry works but gives negative X value for moving forwards

    // Serial.print("Encoder Odometry X: ");
    // Serial.println(encoder_odometry.getX());
    // Serial.print("Encoder Odometry Y: ");
    // Serial.println(encoder_odometry.getY());
    // Serial.print("Encoder Odometry H: ");
    // Serial.println(encoder_odometry.getH());

    // delay(3000);


    // controller.compute()
    //     float currentPosition = encoderL.getRotation();
    
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
