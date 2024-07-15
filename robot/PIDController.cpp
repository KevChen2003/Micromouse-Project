#include "PIDController.hpp"
#include <math.h>

mtrn3100::PIDController::PIDController(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {
    integral = 0;
    derivative = 0;
}

void mtrn3100::PIDController::tune(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
}

// This must be called before trying to achieve a setpoint.
// The first argument becomes the new zero reference point.
// Target is the setpoint value.
void mtrn3100::PIDController::zeroAndSetTarget(float zero, float target) {
    prev_time = micros();
    zero_ref = zero;
    setpoint = target;
}

// Function used to return the last calculated error. 
// The error is the difference between the desired position and current position. 
float mtrn3100::PIDController::getError() {
    return error;
}

// Compute the output signal required from the current value.
float mtrn3100::PIDController::compute(float input) {
    curr_time = micros(); // micros returns the current value of the microcontroller's timer in ms
    dt = static_cast<float>(curr_time - prev_time) / 1e6; // Calculates the elapsed time since the last computation in seconds 
    prev_time = curr_time;

    error = (input - zero_ref) - setpoint; // System error - minus initial encoder inaccuracy from input

    integral = integral + error * dt; // Calculates integral
    derivative = (error - prev_error)/dt; // Calculates derivative
    
    output = Kp * error + Ki * integral + Kd * derivative; // PID formula

    prev_error = error;

    return output;
}