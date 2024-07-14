// OLED not done
// PID controller not done
// implement the simple movement functions
// hardware: IMU broken, needs checking

#include "Wire.h"
#include "motor.hpp"
#include "dualEncoder.hpp"
#include "encoderOdometry.hpp"
#include "lidar.hpp"
#include <VL6180X.h>
#include "IMU.hpp"
#include <MPU6050_light.h> // MPU6050 IMU
#include "OLED.hpp"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // SSD1306 OLED Display
#include <SPI.h>

#define EN_1_A 2 // Pin A for DC motor encoder 1
#define EN_1_B 7 // Pin B for DC motor encoder 1
#define EN_2_A 3 // Pin A for DC motor encoder 2
#define EN_2_B 8 // Pin B for DC motor encoder 2

#define leftLidar_pin A0
#define frontLidar_pin A1
#define rightLidar_pin A2

#define SCREEN_WIDTH 128 // OLED display width in pixels
#define SCREEN_HEIGHT 64 // OLED display height in pixels
#define OLED_RESET -1 // Reset pin number (or -1 if sharing Arduino reset pin)

VL6180X leftLidar, frontLidar, rightLidar; // Creates lidar 1, lidar 2 & lidar 3
const float WALL_DETECTION = 80;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Creates an OLED display
MPU6050 mpu(Wire); // Creates an IMU

// Moving Average Filter for Yaw
unsigned long timer = 0;
const int numReadings = 5; // Number of readings to average
float yawReadings[numReadings]; // Array to store yaw readings
int index = 0; // Index to track position in the array

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(16,90); // Wheel radius & axial length
mtrn3100::Motor L_Motor(9, 10); // correct
mtrn3100::Motor R_Motor(11, 12);
mtrn3100::Lidar lidar;
// mtrn3100::IMU imu;
// mtrn3100::OLED oled;

void setup() {
    // Initialization code for components
    Wire.begin();
    Serial.begin(9600);

    lidar.setupLidars(leftLidar, frontLidar, rightLidar, leftLidar_pin, frontLidar_pin, rightLidar_pin);
    // imu.setupIMU(mpu);
    // oled.setupOLED(display);
}

void loop() {
    delay(50);
    // Update encoder odometry
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());

    // L_Motor.setPWM(100); 
    // R_Motor.setPWM(-100);
    

    // Update Lidar sensor readings
    lidar.updateLidars(leftLidar, frontLidar, rightLidar);

    float frontLidarVal = frontLidar.readRangeSingleMillimeters();
    if (frontLidarVal <= WALL_DETECTION) {
      stopMotors();
      Serial.println("Wall detected! Stopping.");
      return;
    }
    else {
      L_Motor.setPWM(100); 
      R_Motor.setPWM(-100);
    }
  

    // Print position information (optional)
    Serial.print("X : ");
    Serial.print(encoder_odometry.getX());
    Serial.print(" | ");
    Serial.print("Y : ");
    Serial.print(encoder_odometry.getY());
    Serial.print(" | ");
    Serial.print("H : ");
    Serial.print(encoder_odometry.getH());
    Serial.print('\n');
    
    // imu.updateIMU(mpu, yawReadings, numReadings, index, timer, display);

    // Add a delay to control loop rate
    // delay(1500);
};

void stopMotors(){
  L_Motor.setPWM(0); 
  R_Motor.setPWM(0);
}