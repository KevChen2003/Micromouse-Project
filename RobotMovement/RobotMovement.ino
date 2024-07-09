#include "Wire.h"
#include "DualEncoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "EncoderOdometry.hpp"
#include "IMUOdometry.hpp"
#include <MPU6050_light.h> // MPU6050 IMU
#include <VL6180X.h> // VL6180X lidars
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // SSD1306 OLED Display
#include <SPI.h>

#define EN_1_A 2 // Pin A for DC motor encoder 1
#define EN_1_B 7 // Pin B for DC motor encoder 1
#define EN_2_A 3 // Pin A for DC motor encoder 2
#define EN_2_B 8 // Pin B for DC motor encoder 2

#define SCREEN_WIDTH 128 // OLED display width in pixels
#define SCREEN_HEIGHT 64 // OLED display height in pixels
#define OLED_RESET -1 // Reset pin number (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D // See OLED datasheet for address (0x3D for 128x64, 0x3C for 128x32)

MPU6050 mpu(Wire); // Creates an IMU

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Creates an OLED display

VL6180X leftLidar; // Creates lidar 1
VL6180X frontLidar; // Creates lidar 2
VL6180X rightLidar; // Creates lidar 3

#define leftLidar_pin A0; // leftLidar pin number 
#define frontLidar_pin A1; // frontLidar pin number
#define rightLidar_pin A2; // rightLidar pin number
const float WALL_DETECTION = 80 // 80mm threshod for staying at the centre of the cell

// PID Control
// const int Kp = 200;
// const int Ki = 25;
// const int Kd = 0;

const float BasePwm = 200;
// Moving Average Filter
// unsigned long timer = 0;
// const int numReadings = 5; // Number of readings to average
// float yawReadings[numReadings]; // Array to store yaw readings
// int index = 0; // Index to track position in the array

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(16,90); // Wheel radius & axial length
mtrn3100::IMUOdometry IMU_odometry;
mtrn3100::Motor leftMotor(11, 12);
mtrn3100::Motor rightMotor(9, 10);

// need tuning
mtrn3100::PIDController L_PID(0, 0, 0);
mtrn3100::PIDController R_PID(0, 0, 0);

void setup() {
    Serial.begin(115200); // Baud rate
    Wire.begin();

    // IMU setup
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // Stops everything if it could not connect to the MPU6050
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true); // Calculates offsets for gyroscope and accelerometer
    Serial.println("Done!\n");

    // OLED display setup
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    display.display(); // Adafruit splash screen buffer content
    delay(2000);
    display.clearDisplay(); // Clears the buffer

    // Lidar setup
    pinMode(leftLidar_pin, OUTPUT); // Initialises leftLidar_pin as an output 
    pinMode(frontLidar_pin, OUTPUT); // Initialises frontLidar_pin as an output
    pinMode(rightLidar_pin, OUTPUT); // Initialises rightLidar_pin as an output
    digitalWrite(leftLidar_pin, LOW); // Initially disables leftLidar
    digitalWrite(frontLidar_pin, LOW); // Initially disables frontLidar
    digitalWrite(rightLidar_pin, LOW); // Initially disables rightLidar
 
    // Avoids address conflict between lidars
    digitalWrite(leftLidar_pin, HIGH); // Enables leftLidar first
    delay(50);
    leftLidar.init();
    leftLidar.configureDefault();
    leftLidar.setTimeout(250);
    leftLidar.setAddress(0x54); // Changes the address of leftLidar (from 0x29)

    delay(50);

    digitalWrite(frontLidar_pin, HIGH); // Enables frontLidar second
    delay(50);
    frontLidar.init();
    frontLidar.configureDefault();
    frontLidar.setTimeout(250);
    frontLidar.setAddress(0x56); // Changes the address of frontLidar (from 0x29)

    delay(50);

    digitalWrite(rightLidar_pin, HIGH); // Enables rightLidar third
    delay(50);
    rightLidar.init();
    rightLidar.configureDefault();
    rightLidar.setTimeout(250);
    rightLidar.setAddress(0x58); // Changes the address of frontLidar (from 0x29)
  
    // PID Control (Move 100mm)
    L_PID.zeroAndSetTarget(encoder.getLeftRotation(), 2*PI); // getLeftRotation returns value in revolutions
    R_PID.zeroAndSetTarget(encoder.getRightRotation(), -2*PI); // Since the motors face opposite directions
}

void loop() {
    delay(10);
 
    float frontLidarVal = frontLidar.readRangeSingleMillimeters();
     if (frontLidarVal <= WALL_DETECTION) {
        stopMotors();
        Serial.println("Wall detected! Stopping.");
        return;
    }
 
    float PIDLeft = L_PID.compute(leftLidar.readRangeSingleMillimeters());
    float PIDRight = R_PID.compute(rightLidar.readRangeSingleMillimeters()); 
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
    float travelledDist = encoder_odometry.getDistance();
    float adjustedPWM = BasePwm * (250 - travelledDist) / 250;
    Serial.print("Adjusted PWM: ");
    Serial.println(adjustedPWM);
 
    leftMotor.setPWM(adjustedPWM + PIDLeft);
    rightMotor.setPWM(-(adjustedPWM - PIDRight)); // Negative for forward direction
 
    Serial.print("ODOM:\t\t");
    Serial.print(encoder_odometry.getX());
    Serial.print(",\t\t");
    Serial.print(encoder_odometry.getY());
    Serial.print(",\t\t");
    Serial.print(encoder_odometry.getH());
    Serial.println();
}

void stopMotors() {
  leftMotor.setPWM(0);
  rightMotor.setPWM(0);
}

// void turnLeft(int turns) {
//   leftMotor.setPWM()
// }
