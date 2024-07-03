#include "Wire.h"
#include "dualEncoder.hpp"
#include "motor.hpp"
#include "PIDController.hpp"
#include "encoderOdometry.hpp"
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

VL6180X lidar1; // Creates lidar 1
VL6180X lidar2; // Creates lidar 2
VL6180X lidar3; // Creates lidar 3

int lidar1_pin = A0; // lidar1 pin number 
int lidar2_pin = A1; // lidar2 pin number
int lidar3_pin = A2; // lidar3 pin number

// PID Control
const int Kp = 200;
const int Ki = 25;
const int Kd = 0;

// Moving Average Filter
unsigned long timer = 0;
const int numReadings = 5; // Number of readings to average
float yawReadings[numReadings]; // Array to store yaw readings
int index = 0; // Index to track position in the array

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(16,90); // Wheel radius & axial length
mtrn3100::IMUOdometry IMU_odometry;
mtrn3100::Motor L_Motor(11, 12);
mtrn3100::Motor R_Motor(9, 10);
mtrn3100::PIDController L_PID(Kp, Ki, Kd);
mtrn3100::PIDController R_PID(Kp, Ki, Kd);

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
    pinMode(lidar1_pin, OUTPUT); // Initialises lidar1_pin as an output 
    pinMode(lidar2_pin, OUTPUT); // Initialises lidar2_pin as an output
    pinMode(lidar3_pin, OUTPUT); // Initialises lidar3_pin as an output
    digitalWrite(lidar1_pin, LOW); // Initially disables lidar1
    digitalWrite(lidar2_pin, LOW); // Initially disables lidar2
    digitalWrite(lidar3_pin, LOW); // Initially disables lidar3
 
    // Avoids address conflict between lidars
    digitalWrite(lidar1_pin, HIGH); // Enables lidar1 first
    delay(50);
    lidar1.init();
    lidar1.configureDefault();
    lidar1.setTimeout(250);
    lidar1.setAddress(0x54); // Changes the address of lidar1 (from 0x29)

    delay(50);

    digitalWrite(lidar2_pin, HIGH); // Enables lidar2 second
    delay(50);
    lidar2.init();
    lidar2.configureDefault();
    lidar2.setTimeout(250);
    lidar2.setAddress(0x56); // Changes the address of lidar2 (from 0x29)

    delay(50);

    digitalWrite(lidar3_pin, HIGH); // Enables lidar3 third
    delay(50);
    lidar3.init();
    lidar3.configureDefault();
    lidar3.setTimeout(250);
    lidar3.setAddress(0x58); // Changes the address of lidar2 (from 0x29)
  
    // PID Control (Move 100mm)
    L_PID.zeroAndSetTarget(encoder.getLeftRotation(), 2*PI); // getLeftRotation returns value in revolutions
    R_PID.zeroAndSetTarget(encoder.getRightRotation(), -2*PI); // Since the motors face opposite directions
}

// Function to update the OLED with the moving yaw average
void updateDisplay(float yawAverage) {
  display.clearDisplay();
  display.setTextSize(1); // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // White text
  display.setCursor(0,0); // Starts at the top-left corner
  display.print(F("Yaw (Moving Average): ")); // Since we are reusing this text, it is better to allocate it to flash memory instead of SRAM
  display.println(yawAverage);
  display.display();
}

void loop() {
    mpu.update();
    if (millis() - timer > 1000) { // Prints data every second
        Serial.print(F("ACCELERO  X: "));
        Serial.print(mpu.getAccX());
        Serial.print("\tY: ");
        Serial.print(mpu.getAccY());
        Serial.print("\tZ: ");
        Serial.println(mpu.getAccZ());
    
        Serial.print(F("GYRO      X: "));
        Serial.print(mpu.getGyroX());
        Serial.print("\tY: ");
        Serial.print(mpu.getGyroY());
        Serial.print("\tZ: ");
        Serial.println(mpu.getGyroZ());
    
        Serial.print(F("ACC ANGLE X: "));
        Serial.print(mpu.getAccAngleX());
        Serial.print("\tY: ");
        Serial.println(mpu.getAccAngleY());
        
        Serial.print(F("ANGLE     X: "));
        Serial.print(mpu.getAngleX());
        Serial.print("\tY: ");
        Serial.print(mpu.getAngleY());
        Serial.print("\tZ: ");
        Serial.println(mpu.getAngleZ());

        float currentYaw = mpu.getAngleZ(); // Gets current yaw reading

        // Updates the array with new yaw reading
        yawReadings[index] = currentYaw;
        index = (index + 1) % numReadings; // Increments index and wrap around if needed

        // Calculates the average of the last numReadings yaw readings
        float yawAverage = 0;
        for(int i = 0; i < numReadings; i++) {
        yawAverage += yawReadings[i];
        }
        yawAverage /= numReadings;
        
        // Prints moving yaw average to serial monitor
        Serial.print(F("Yaw (Moving Average): "));
        Serial.println(yawAverage);
        Serial.println(F("=====================================================\n"));
        timer = millis();

        // Calls the function to update the OLED with the yaw average value
        updateDisplay(yawAverage);
    }

    // Lidar data to serial monitor
    Serial.print(lidar1.readRangeSingleMillimeters()); // Prints the range value of lidar1 to serial monitor
    Serial.print(" | ");
    Serial.print(lidar2.readRangeSingleMillimeters()); // Prints the range value of lidar2 to serial monitor
    Serial.println();
    if (lidar1.timeoutOccurred()) { 
        Serial.print("lidar 1 TIMEOUT");
    }
    if (sensor2.timeoutOccurred()) { 
        Serial.print("lidar 2 TIMEOUT"); 
    }
    delay(100); // Waits 0.1 second for next scan


    // PID Control
    delay(15);
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

    Serial.print("X : ");
    Serial.println(encoder_odometry.getX());
    Serial.print("Y : ");
    Serial.println(encoder_odometry.getY());
    
    // Compute PID Control
    L_Motor.setPWM(L_PID.compute(encoder.getLeftRotation()));
    R_Motor.setPWM(R_PID.compute(encoder.getRightRotation()));
}
