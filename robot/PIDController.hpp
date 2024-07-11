#pragma once

#include <Arduino.h>

namespace mtrn3100 {
    class PIDController {
    public:
        PIDController(float Kp, float Ki, float Kd);
        void tune(float p, float i, float d);
        void zeroAndSetTarget(float zero, float target);
        float getError();
        float compute(float input);

        uint32_t prev_time, curr_time = micros();
        float dt;
    private:
        float Kp, Ki, Kd;
        float error, derivative, integral, output;
        float prev_error = 0;
        float setpoint = 0;
        float zero_ref = 0;
    };
}