#pragma once

#include <Arduino.h>
#include "math.h"

namespace mtrn3100 {
    class Motor {
    public:
        Motor(uint8_t pwm_pin, uint8_t in2);
        void setPWM(int16_t pwm);
    private:
        const uint8_t pwm_pin;
        const uint8_t dir_pin;
    };
}


