#include "Motor.hpp"
#include <Arduino.h>
#include "math.h"

mtrn3100::Motor::Motor(uint8_t pwm_pin, uint8_t in2) : pwm_pin(pwm_pin), dir_pin(in2) {
    pinMode(this->dir_pin, OUTPUT);
    pinMode(this->pwm_pin, OUTPUT);
}

// This function outputs the desired motor direction and the PWM signal. 
// NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.
void mtrn3100::Motor::setPWM(int16_t pwm) {
    if (pwm < 0) {
        digitalWrite(this->dir_pin, LOW); // direction pin is an interrupt pin
        pwm = -pwm;
    } else {
        digitalWrite(this->dir_pin, HIGH);
    }
    // TODO: Output digital direction pin based on if input signal is positive or negative.
    // TODO: Output PWM signal between 0 - 255.
    // int new_pwm = max(min(pwm, 255), 0);
    analogWrite(this->pwm_pin, pwm);
}