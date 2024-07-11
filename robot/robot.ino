#include "Wire.h"
#include "IMU.hpp"
#include "OLED.hpp"
#include "lidar.hpp"
#include "motor.hpp"
#include "dualEncoder.hpp"
#include "encoderOdometry.hpp"
#include "PIDController.hpp"
//#include "IMUOdometry.hpp" <-- for turning
#include <MPU6050_light.h> // MPU6050 IMU
#include <VL6180X.h> // VL6180X lidars
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // SSD1306 OLED Display
#include <SPI.h>

#define EN_1_A 2 // Pin A for DC motor encoder 1
#define EN_1_B 7 // Pin B for DC motor encoder 1
#define EN_2_A 3 // Pin A for DC motor encoder 2
#define EN_2_B 8 // Pin B for DC motor encoder 2

#define lidar1_pin A3
#define lidar2_pin A2
#define lidar3_pin A4

#define SCREEN_WIDTH 128 // OLED display width in pixels
#define SCREEN_HEIGHT 64 // OLED display height in pixels
#define OLED_RESET -1 // Reset pin number (or -1 if sharing Arduino reset pin)

MPU6050 mpu(Wire); // Creates an IMU
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Creates an OLED display
VL6180X lidar1, lidar2, lidar3; // Creates lidar 1, lidar 2 & lidar 3

// PID Control
const int Kp = 200;
const int Ki = 25;
const int Kd = 10;

// Moving Average Filter for Yaw
unsigned long timer = 0;
const int numReadings = 5; // Number of readings to average
float yawReadings[numReadings]; // Array to store yaw readings
int index = 0; // Index to track position in the array

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(16,90); // Wheel radius & axial length
//mtrn3100::IMUOdometry IMU_odometry; for turning
mtrn3100::Motor L_Motor(11, 12);
mtrn3100::Motor R_Motor(9, 10);
mtrn3100::PIDController L_PID(Kp, Ki, Kd);
mtrn3100::PIDController R_PID(Kp, Ki, Kd);

void setup() {
    // Initialization code for components
    Serial.begin(9600);

    Serial.println('hello');

    Wire.begin();

    // Initialize IMU, OLED, and Lidars (as per your setup)
    IMU::setupIMU(mpu);
    OLED::setupOLED(display);
    Lidar::setupLidars(lidar1, lidar2, lidar3, lidar1_pin, lidar2_pin, lidar3_pin);

    // Set initial target positions for PID control (example: move forward 100mm)
    L_PID.zeroAndSetTarget(encoder.getLeftRotation(), 2 * PI);
    R_PID.zeroAndSetTarget(encoder.getRightRotation(), -2 * PI);
}

void loop() {
    // Update IMU readings and display

    Serial.println('1');

    IMU::updateIMU(mpu, yawReadings, numReadings, index, timer, display);

    Serial.println('2');

    // Update Lidar sensor readings
    Lidar::updateLidars(lidar1, lidar2, lidar3);

    Serial.println('3');

    // Update encoder odometry
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());

    // Compute PID output and control motors
    float L_output = L_PID.compute(encoder.getLeftRotation());
    float R_output = R_PID.compute(encoder.getRightRotation());

    L_Motor.setPWM(L_output);
    R_Motor.setPWM(R_output);

    // Print position information (optional)
    Serial.print("X : ");
    Serial.println(encoder_odometry.getX());
    Serial.print("Y : ");
    Serial.println(encoder_odometry.getY());

    // Add a delay to control loop rate
    delay(15);
}

