#pragma once 

#include <math.h>

namespace mtrn3100 {
class PIDController {
    public:
        PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {
            integral = 0;
            derivative = 0;
        }
    
        // want to get the distance between wall and robot. 
        // must read off lidars 

        // Compute the output signal required from the current value.
        float compute(float input) {
        
            curr_time = micros(); // micros returns the current value of the microcontroller's timer in ms
            dt = static_cast<float>(curr_time - prev_time) / 1e6; // Calculates the elapsed time since the last computation in seconds 
            prev_time = curr_time;

            error = (input - zero_ref) - setpoint; // System error

            integral = prev_integral + error * dt; // Calculates integral
            derivative = (error-prev_error) / dt; // Calculates derivative

            // if (abs(error) > 0.2) {
            //     output = kp * error + ki * integral + kd * derivative; // PID formula
            // } else {
            //     return 0.0;
            // }

            output = kp * error + ki * integral + kd * derivative; // PID formula

            prev_error = error;
            prev_integral = integral;

            return output;
        }

        void tune(float p, float i, float d) {
            kp = p;
            ki = i;
            kd = d;
        }

        // Function used to return the last calculated error. 
        // The error is the difference between the desired position and current position. 
        float getError() {
            return error;
        }

        // This must be called before trying to achieve a setpoint.
        // The first argument becomes the new zero reference point.
        // Target is the setpoint value.
        void zeroAndSetTarget(float zero, float target) {
            prev_time = micros();
            zero_ref = zero;
            setpoint = target;
        }
        
    public:
        uint32_t prev_time, curr_time = micros();
        float dt;

    private:
        float kp;
        float ki;
        float kd;
        float error, derivative, integral, output;
        float prev_error = 0;
        float setpoint = 0;
        float zero_ref = 0; 
        float prev_integral = 0;

};
}