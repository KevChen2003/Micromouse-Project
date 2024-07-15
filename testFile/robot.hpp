// #pragma once

#ifndef ROBOT_H
#define ROBOT_H

#include "Wire.h"
#include "motor.hpp"
#include "dualEncoder.hpp"
#include "encoderOdometry.hpp"
#include "lidar.hpp"
#include <VL6180X.h>
#include "IMU.hpp"
#include <MPU6050_light.h> // MPU6050 IMU
// #include "OLED.hpp"
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h> // SSD1306 OLED Display
#include <SPI.h>
#include "PIDController.hpp"

#define EN_1_A 2 // Pin A for DC motor encoder 1
#define EN_1_B 7 // Pin B for DC motor encoder 1
#define EN_2_A 3 // Pin A for DC motor encoder 2
#define EN_2_B 8 // Pin B for DC motor encoder 2

#define leftLidar_pin A0
#define rightLidar_pin A2
#define frontLidar_pin A1

#define SCREEN_WIDTH 128 // OLED display width in pixels
#define SCREEN_HEIGHT 64 // OLED display height in pixels
#define OLED_RESET -1 // Reset pin number (or -1 if sharing Arduino reset pin)

const float WALL_DETECTION = 80;
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Creates an OLED display
MPU6050 mpu(Wire); // Creates an IMU

// Moving Average Filter for Yaw
unsigned long timer = 0;
const int numReadings = 5; // Number of readings to average
float yawReadings[numReadings]; // Array to store yaw readings
int index = 0; // Index to track position in the array

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(16,90); // Wheel radius & axial length
mtrn3100::Motor R_Motor(9, 10); // correct
mtrn3100::Motor L_Motor(11, 12);
mtrn3100::Lidar lidar;
mtrn3100::IMU imu;
// mtrn3100::OLED oled;

// tune pids

// tuned to 1 cell, a bit slow
// left: 9,0,0.0012
// right 8,0, 0

mtrn3100::PIDController l_forward_pid(9,0,0.0012);
mtrn3100::PIDController r_forward_pid(8,0,0);

mtrn3100::PIDController side_lidar_pid(0.5,0.1,0);
mtrn3100::PIDController front_lidar_pid(0.5,0,0);


// Function declarations for Movement
void moveForward(int cell_count);
int driveMotors(float l_target, float r_target);

float l_forward_signal = 0;
float r_forward_signal = 0;
float front_signal = 0;

float side_lidar_signal = 0;
float front_lidar_signal = 0;
float l_drive_signal = 0;
float r_drive_signal = 0;
float drive_signal = 0;
float right_signal = 0;
float left_signal = 0;



const int CONTROL_SPEED = 80;
#endif