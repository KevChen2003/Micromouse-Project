#pragma once

#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {
class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), L(wheelBase), lastLPos(0), lastRPos(0) {}

    //TODO: COMPLETE THIS FUNCTION
    void update(float leftValue,float rightValue) {

        //TODO: Calculate the change in radians since the last update.
        float delta_left_radians = leftValue - lastLPos;   // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEYARE NOT THE WRONG DIRECTION 
        float delta_right_radians = rightValue - lastRPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEY ARE NOT THE WRONG DIRECTION 

        float delta_s = (R / 2) * (delta_left_radians + delta_right_radians);
        float delta_theta = (R / (2 * L)) * (-delta_left_radians + delta_right_radians);

        

        //TODO: Calculate the foward kinematics
        x += delta_s * cos(h);
        y += delta_s * sin(h);
        h += delta_theta;
        lastLPos = leftValue;
        lastRPos = rightValue;
    }

    float getRotation(float count) {
        float revolutions = static_cast<float>(count) / counts_per_revolution;
        float radians = revolutions * 2.0 * PI;
        return radians;
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getH() const { return h; }

private:
    float x, y, h;
    const float R, L;
    float lastLPos, lastRPos;
    uint16_t counts_per_revolution = 690;
};

}

