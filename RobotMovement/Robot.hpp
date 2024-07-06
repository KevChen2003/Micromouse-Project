#pragma once
#include <Arduino.h>
#include <VL6180X.h>
#include <MPU6050_light.h>
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
          uint8_t left_encA, uint8_t left_encB, uint8_t right_encA, uint8_t right_encB,
          float radius, float axle, uint8_t imuAddr) 
        : dualEncoder(left_encA, left_encB, right_encA, right_encB), imuOdometry(), encoderOdometry(radius, axle), 
          motor_left(leftMotorPWM, leftMotorDir), motor_right(rightMotorPWM, rightMotorDir),
          pidController(10, 0, 0), left_pid(0,0,0), right_pid(0,0,0), imu_address(imuAddr) {}

    // Function to initialize components and calibrate sensors
    void initialize() {
        // Initialize IMU (if necessary)
        // imu.begin(imu_address);

        // Zero and set target position for PID controller
        pidController.zeroAndSetTarget(0.0, 0.0);

        // Any other initialization code
    }

    // Function to move the robot straight for a specified distance (in meters)

    // // changing to do it based on cell count
    // void moveStraight(float cell_count, int pwm) {     
    //     dualEncoder.reset(); // initialise to 0
    //     // float target_radians = cell_count * radians_per_cell; // setpoint in radians
    //     // float target_radians = cell_count * radians_per_cell; // setpoint in radians

    //     float target_radians = cell_count * radians_per_cell; // setpoint in radians
    //     float target_counts = target_radians * counts_per_revolution / (2 * PI);

    //     // l_forward_pid.zeroEncoderAndSetTarget(dualEncoder.l_count, target_counts); // setting zero and target idk
    //     // r_forward_pid.zeroEncoderAndSetTarget(dualEncoder.r_count, target_counts);
    //     pidController.zeroAndSetTarget(0, target_counts);

    //     while (abs(dualEncoder.l_count) < abs(target_counts) || abs(dualEncoder.r_count) < abs(target_counts)) { // target not reached
    //             // motor offset correction (not rly sure about this)
    //             float pidOutput = pidController.compute((dualEncoder.l_count + dualEncoder.r_count) / 2.0);
    //             int leftPWM = pwm + pidOutput;
    //             int rightPWM = -(pwm - pidOutput);
    //             leftMotor.setPWM(leftPWM);
    //             rightMotor.setPWM(rightPWM);
    //             delay(10); // Adjust delay as necessary
    //         }
    //         stopMotors();
    //     }

    void moveStraight(float cell_count, int pwm) {
        dualEncoder.reset();

        float target_radians = cell_count * radians_per_cell; // target in radians
        float target_counts = target_radians * counts_per_radian; // convert target to counts

        pidController.zeroAndSetTarget((dualEncoder.l_count + dualEncoder.r_count) / 2.0, target_counts);

        while (abs(dualEncoder.l_count) < abs(target_counts) || abs(dualEncoder.r_count) < abs(target_counts)) {
            float pidOutput = pidController.compute((dualEncoder.l_count + dualEncoder.r_count) / 2.0);

            int leftPWM = pwm + pidOutput;
            int rightPWM = -(pwm - pidOutput);

            leftMotor.setPWM(leftPWM);
            rightMotor.setPWM(rightPWM);

            delay(10);
        }

        stopMotors();
    }



      void turnLeft(int turn_count, int pwm) {
            float targetAngle = turn_count * (PI / 2.0);
            float targetCounts = targetAngle * counts_per_radian; // convert target to counts

            dualEncoder.reset();

            pidController.zeroAndSetTarget((dualEncoder.l_count + dualEncoder.r_count) / 2.0, targetCounts);

            while (abs(dualEncoder.l_count) < abs(targetCounts) || abs(dualEncoder.r_count) < abs(targetCounts)) {
                float pidOutput = pidController.compute((dualEncoder.l_count + dualEncoder.r_count) / 2.0);

                int leftPWM = pwm - pidOutput;
                int rightPWM = pwm + pidOutput;

                leftMotor.setPWM(leftPWM);
                rightMotor.setPWM(rightPWM);

                delay(10); // Adjust delay as necessary
            }

            stopMotors();
        }

void turnRight(int turn_count, int baseMotorSpeed) {
            float targetAngle = turn_count * (PI / 2.0);
            float targetCounts = targetAngle * counts_per_radian; // convert target to counts

            encoders.l_count = 0;
            encoders.r_count = 0;

            pidController.zeroAndSetTarget((encoders.l_count + encoders.r_count) / 2.0, targetCounts);

            while (abs(encoders.l_count) < abs(targetCounts) || abs(encoders.r_count) < abs(targetCounts)) {
                float pidOutput = pidController.compute((encoders.l_count + encoders.r_count) / 2.0);

                int leftPWM = baseMotorSpeed + pidOutput;
                int rightPWM = baseMotorSpeed - pidOutput;

                leftMotor.setPWM(-leftPWM);
                rightMotor.setPWM(-rightPWM);
                delay(10);
            }
            stopMotors();
}

    // Function to execute a sequence of movements based on commands
    void executeMovementSequence(const String& commands) {
        for (char command : commands) {
            switch (command) {
                case 'f':  // Forward
                    moveStraight(cellSize, pwm);
                    break;
                case 'l':  // Left turn
                    turnLeft(90.0, pwm);
                    break;
                case 'r':  // Right turn
                    turnRight(90.0, pwm);
                    break;
                default:
                    break;
            }
        }
    }


    // // Function to turn the robot left by a specified turn_count (in degrees)
    // void turnLeft(float turn_count, int pwm) {
    //     // Convert turn_count to radians
    //     float targetturn_count = turn_count * PI / 180.0;

    //     // Reset dualEncoder
    //     dualEncoder.l_count = 0;
    //     dualEncoder.r_count = 0;

    //     // Turn left until target turn_count is reached
    //     while (abs(encoderOdometry.getH()) < abs(targetturn_count)) {
    //         // Set motor speeds for turning left
    //         motor_left.setPWM(pwm);  // Adjusted for opposite directions
    //         motor_right.setPWM(pwm);  // Adjusted for opposite directions
    //     }

    //     // Stop motors
    //     stopMotors();
    // }

    // // Function to turn the robot right by a specified turn_count (in degrees)
    // void turnRight(float turn_count, int pwm) {
    //     // Convert turn_count to radians
    //     float targetturn_count = turn_count * PI / 180.0;

    //     // Reset dualEncoder
    //     dualEncoder.l_count = 0;
    //     dualEncoder.r_count = 0;

    //     // Turn right until target turn_count is reached
    //     while (abs(encoderOdometry.getH()) < abs(targetturn_count)) {
    //         // Set motor speeds for turning right
    //         motor_left.setPWM(-pwm);  // Adjusted for opposite directions
    //         motor_right.setPWM(-pwm);  // Adjusted for opposite directions
    //     }

    //     // Stop motors
    //     stopMotors();
    // }

    // // Function to execute a sequence of movements based on commands
    // void executeMovementSequence(const String& commands) {
    //     for (char command : commands) {
    //         switch (command) {
    //             case 'f':  // Forward
    //                 moveStraight(cellSize, pwm);
    //                 break;
    //             case 'l':  // Left turn
    //                 turnLeft(90.0, pwm);
    //                 break;
    //             case 'r':  // Right turn
    //                 turnRight(90.0, pwm);
    //                 break;
    //             default:
    //                 break;
    //         }
    //     }
    // }

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
    PIDController left_pid;
    PIDController right_pid;
    EncoderOdometry encoderOdometry;
    Motor motor_left;
    Motor motor_right;

    // Configuration
    int pwm = 0; // Base motor speed for driving straight
    const uint8_t imu_address;
    const float radius = 16;
    const float axle = 90;
    const float radiansPer90Turn;
    const float cell_size = 250;
    const float radians_per_cell = cell_size / radius;

};

}  // namespace mtrn3100
// distance per revolution = perimeter = 100.5310mm
// cell size = 250mm
// revolutions per cell = cell size / perimeter = 2.4868
// counts per cell = revolutions per cell * 700 = 1740.76 counts
// radians per cell = 1740*2*PI/700 = 15.625mm which is basically cell size/radius 
// assuming it moves from centre of cell 1 to centre of cell 5, target cells = 4 cells
// target distance = radians_per_cell * target cells

