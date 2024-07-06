// // stub motor testing
// #include <Wire.h>
// #include "Robot.hpp"


// #define LEFT_MOTOR_PWM 9
// #define LEFT_MOTOR_DIR 10
// #define RIGHT_MOTOR_PWM 11
// #define RIGHT_MOTOR_DIR 12

// #define LEFT_ENCODER_A 2
// #define LEFT_ENCODER_B 7
// #define RIGHT_ENCODER_A 3
// #define RIGHT_ENCODER_B 8


// #define LIDAR_FRONT_PIN A3 
// #define LIDAR_LEFT_PIN A2 
// #define LIDAR_RIGHT_PIN A4 


// const float WHEEL_RADIUS = 16; 
// const float WHEEL_BASE = 90;  


// mtrn3100::Robot robot(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR, RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR,
//                       LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B,
//                       WHEEL_RADIUS, WHEEL_BASE);

// void setup() {
//     Serial.begin(9600);
// }





// void loop() {
//     // const char* commands = "f";
//     // robot.setCurrentPWM(200); 
//     // robot.chainMovements(commands);
//     // while (true) {
//     //     robot.stopMotors();
//     //     delay(1000);
//     // }
//     robot.moveForward(50);
//     delay(3000);
//     robot.stopMotors();
// }



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
    // robot.initialize();
}

void loop() {
    // Turn right by 90 degrees
    // robot.turnLeft(90,170);
    // robot.turnRight(PI/2);

    // Delay to observe the turn (adjust as needed)
    // delay(2000);

    // Stop motors before next operation
    // robot.stopMotors();

    // Optional: Perform other operations or move to next task
    // For example:
    // robot.moveStraight(0.5);  // Move straight for 0.5 meters
    // delay(2000);  // Delay between movements

    float left_rotation = left_encoder.getRotation();
    float right_rotation = right_encoder.getRotation();

    Serial.print("left rotation");
    Serial.println(left_rotation);
    
    Serial.print("right rotation");
    Serial.println(right_rotation);
    delay(1000);
}



// #include "motor.hpp"

// // Define motor pins
// const uint8_t motor1_pwm_pin = 9;  // Example PWM pin for Motor 1
// const uint8_t motor1_dir_pin = 10;  // Example direction pin for Motor 1
// const uint8_t motor2_pwm_pin = 11;  // Example PWM pin for Motor 2
// const uint8_t motor2_dir_pin = 12;  // Example direction pin for Motor 2

// // Create instances of Motor class
// mtrn3100::Motor motor1(motor1_pwm_pin, motor1_dir_pin);
// mtrn3100::Motor motor2(motor2_pwm_pin, motor2_dir_pin);

// void setup() {
//     // Set up serial communication if needed
//     Serial.begin(9600);

//     // Initialize motors
//     // Assuming you've set up pins 3, 4, 5, and 6 as PWM and direction pins
//     // These pins should be connected to the motor driver board or directly to the motors
// }

// void loop() {
//     // Move both motors forward at 50 PWM speed
//     motor1.setPWM(70);
//     motor2.setPWM(-70);

//     // You can add delays or other code here as needed
//     delay(1000); // Example: move forward for 1 second

//     // Stop motors (if needed)
//     motor1.setPWM(0);
//     motor2.setPWM(0);

//     // You can add more functionality or control logic here
// }

