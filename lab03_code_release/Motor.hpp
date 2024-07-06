#pragma once

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {

    class Motor {
    public:
        Motor(uint8_t pwm_pin, uint8_t in2) : pwm_pin(pwm_pin), dir_pin(in2) {
            pinMode(pwm_pin, OUTPUT);
            pinMode(dir_pin, OUTPUT);
        }

        void setPWM(int16_t pwm) {
            // Output digital direction pin based on if input signal is positive or negative.
            if (pwm >= 0) {
                digitalWrite(dir_pin, HIGH); // Set direction pin high for forward
            }
            else {
                digitalWrite(dir_pin, LOW); // Set direction pin low for reverse
            }

            // Output PWM signal between 0 - 255.
            analogWrite(pwm_pin, abs(pwm));
        }

    private:
        const uint8_t pwm_pin;
        const uint8_t dir_pin;
    };

}  // namespace mtrn3100
