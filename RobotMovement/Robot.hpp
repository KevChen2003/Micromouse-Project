// // Stub Functions for moving straight, turning left and right
// #pragma once

// #include "Motor.hpp"
// #include "DualEncoder.hpp"
// #include "EncoderOdometry.hpp"
// #include "IMUOdometry.hpp"

// namespace mtrn3100 {
//     class Robot {
//     public:
//         Robot(uint8_t left_pwm_pin, uint8_t left_dir_pin, uint8_t right_pwm_pin, uint8_t right_dir_pin,
//               uint8_t enc1, uint8_t enc2, uint8_t enc3, uint8_t enc4, float radius, float wheelBase)
//             : leftMotor(left_pwm_pin, left_dir_pin), rightMotor(right_pwm_pin, right_dir_pin),
//               encoder(enc1, enc2, enc3, enc4), encoderOdom(radius, wheelBase) {}

//         void moveForward(int pwm) {
//             leftMotor.setPWM(pwm);    
//             rightMotor.setPWM(-pwm); 
//         }

//         void turnLeft(int pwm) {
//             leftMotor.setPWM(pwm);   
//             rightMotor.setPWM(pwm);  
//         }

//         void turnRight(int pwm) {
//             leftMotor.setPWM(-pwm);    
//             rightMotor.setPWM(-pwm);   
//         }

//         void stopMotors() {
//             leftMotor.setPWM(0);
//             rightMotor.setPWM(0);
//         }

//         void chainMovements(const String& movements) {
//             for (char move : movements) {
//                 switch (move) {
//                     case 'f':
//                         moveForward(currentPWM); 
//                         delay(1000); 
//                         break;
//                     case 'l':
//                         turnLeft(currentPWM); 
//                         delay(500); 
//                         break;
//                     case 'r':
//                         turnRight(currentPWM); 
//                         delay(500); 
//                         break;
//                 }
//                 stopMotors();
//             }
//         }

//         void setCurrentPWM(int pwm) {
//             currentPWM = pwm;
//         }

//     private:
//         Motor leftMotor;
//         Motor rightMotor;
//         DualEncoder encoder;
//         EncoderOdometry encoderOdom;
//         int currentPWM = 0; 
//     };
// }





#pragma once

#include <Arduino.h>
#include "DualEncoder.hpp"
#include "Encoder.hpp"
#include "IMUOdometry.hpp"
#include "EncoderOdometry.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

namespace mtrn3100 {
class Robot {
public:
    Robot(uint8_t leftMotorPWM, uint8_t leftMotorDir, uint8_t rightMotorPWM, uint8_t rightMotorDir,
          uint8_t enc1, uint8_t enc2, uint8_t enc3, uint8_t enc4,
          float wheelRadius, float wheelBase, uint8_t imuAddr) 
        : dualEncoder(enc1, enc2, enc3, enc4), imuOdometry(), encoderOdometry(wheelRadius, wheelBase), 
          motor_left(leftMotorPWM, leftMotorDir), motor_right(rightMotorPWM, rightMotorDir),
          pidController(10, 0, 0), imu_address(imuAddr) {}

    // Function to initialize components and calibrate sensors
    void initialize() {
        // Initialize IMU (if necessary)
        // imu.begin(imu_address);

        // Zero and set target position for PID controller
        pidController.zeroAndSetTarget(0.0, 0.0);

        // Any other initialization code
    }

    // Function to move the robot straight for a specified distance (in meters)
    void moveStraight(float distance, int baseMotorSpeed) {
        // Convert distance to encoder counts
        float targetCounts = distance / (2 * PI * wheelRadius) * encoderCountsPerRevolution;

        // Reset encoders
        dualEncoder.l_count = 0;
        dualEncoder.r_count = 0;

        // Drive straight until target distance is reached
        while (abs(dualEncoder.l_count) < targetCounts && abs(dualEncoder.r_count) < targetCounts) {
            // Use PID controller to maintain straight path
            float error = dualEncoder.l_count - dualEncoder.r_count;
            float correction = pidController.compute(error);

            // Set motor speeds (opposite directions for forward motion)
            // int leftSpeed = baseMotorSpeed + correction;
            // int rightSpeed = -baseMotorSpeed - correction;

            motor_left.setPWM(baseMotorSpeed);
            motor_right.setPWM(-baseMotorSpeed);
        }

        // Stop motors
        stopMotors();
    }

    // Function to turn the robot left by a specified angle (in degrees)
    void turnLeft(float angle, int baseMotorSpeed) {
        // Convert angle to radians
        float targetAngle = angle * PI / 180.0;

        // Reset encoders
        dualEncoder.l_count = 0;
        dualEncoder.r_count = 0;

        // Turn left until target angle is reached
        while (abs(encoderOdometry.getH()) < abs(targetAngle)) {
            // Set motor speeds for turning left
            motor_left.setPWM(baseMotorSpeed);  // Adjusted for opposite directions
            motor_right.setPWM(baseMotorSpeed);  // Adjusted for opposite directions
        }

        // Stop motors
        stopMotors();
    }

    // Function to turn the robot right by a specified angle (in degrees)
    void turnRight(float angle, int baseMotorSpeed) {
        // Convert angle to radians
        float targetAngle = angle * PI / 180.0;

        // Reset encoders
        dualEncoder.l_count = 0;
        dualEncoder.r_count = 0;

        // Turn right until target angle is reached
        while (abs(encoderOdometry.getH()) < abs(targetAngle)) {
            // Set motor speeds for turning right
            motor_left.setPWM(-baseMotorSpeed);  // Adjusted for opposite directions
            motor_right.setPWM(-baseMotorSpeed);  // Adjusted for opposite directions
        }

        // Stop motors
        stopMotors();
    }

    // Function to execute a sequence of movements based on commands
    void executeMovementSequence(const String& commands) {
        for (char command : commands) {
            switch (command) {
                case 'f':  // Forward
                    moveStraight(cellSize, baseMotorSpeed);
                    break;
                case 'l':  // Left turn
                    turnLeft(90.0, baseMotorSpeed);
                    break;
                case 'r':  // Right turn
                    turnRight(90.0, baseMotorSpeed);
                    break;
                default:
                    break;
            }
        }
    }

    // Function to update robot's position based on sensor readings
    void updatePosition() {
        // Update position based on IMU data
        float accel_x = 0.0; // Replace with actual accelerometer data
        float accel_y = 0.0; // Replace with actual accelerometer data
        imuOdometry.update(accel_x, accel_y);

        // Update position based on encoder data
        float leftRotation = dualEncoder.getLeftRotation();
        float rightRotation = dualEncoder.getRightRotation();
        encoderOdometry.update(leftRotation, rightRotation);
    }

    void stopMotors(){
      motor_left.setPWM(0);
      motor_right.setPWM(0);
    }

private:
    // Constants and variables
    const uint16_t encoderCountsPerRevolution = 690; // Example value, adjust as per your encoder specs
    const float cellSize = 0.25;  // Each cell is 250mm x 250mm

    // Components
    DualEncoder dualEncoder;
    IMUOdometry imuOdometry;
    PIDController pidController;
    EncoderOdometry encoderOdometry;
    Motor motor_left;
    Motor motor_right;

    // Configuration
    int baseMotorSpeed = 0; // Base motor speed for driving straight
    const uint8_t imu_address;
    const float wheelRadius;
    const float wheelBase;
    const float radiansPer90Turn;
};

}  // namespace mtrn3100

