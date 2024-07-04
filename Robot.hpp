#pragma once

#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "IMUOdometry.hpp"

namespace mtrn3100 {
    class Robot {
    public:
        Robot(uint8_t left_pwm_pin, uint8_t left_dir_pin, uint8_t right_pwm_pin, uint8_t right_dir_pin,
              uint8_t enc1, uint8_t enc2, uint8_t enc3, uint8_t enc4, float radius, float wheelBase)
            : leftMotor(left_pwm_pin, left_dir_pin), rightMotor(right_pwm_pin, right_dir_pin),
              encoder(enc1, enc2, enc3, enc4), encoderOdom(radius, wheelBase) {}

        void moveForward(int pwm) {
            leftMotor.setPWM(pwm);    
            rightMotor.setPWM(-pwm); 
        }

        void turnLeft(int pwm) {
            leftMotor.setPWM(pwm);   
            rightMotor.setPWM(pwm);  
        }

        void turnRight(int pwm) {
            leftMotor.setPWM(-pwm);    
            rightMotor.setPWM(-pwm);   
        }

        void stopMotors() {
            leftMotor.setPWM(0);
            rightMotor.setPWM(0);
        }

        void chainMovements(const String& movements) {
            for (char move : movements) {
                switch (move) {
                    case 'f':
                        moveForward(currentPWM); 
                        delay(1000); 
                        break;
                    case 'l':
                        turnLeft(currentPWM); 
                        delay(500); 
                        break;
                    case 'r':
                        turnRight(currentPWM); 
                        delay(500); 
                        break;
                }
                stopMotors();
            }
        }

        void setCurrentPWM(int pwm) {
            currentPWM = pwm;
        }

    private:
        Motor leftMotor;
        Motor rightMotor;
        DualEncoder encoder;
        EncoderOdometry encoderOdom;
        int currentPWM = 200; 
    };
}
