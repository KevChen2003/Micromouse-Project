#pragma once

#include <Arduino.h>

const double WHEEL_RADIUS = 0.016;
const double AXLE_LEN = 0.1;

namespace mtrn3100 {
class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), L(wheelBase), lastLPos(0), lastRPos(0) {}

    //TODO: COMPLETE THIS FUNCTION
    void update(float leftValue,float rightValue) {

        //TODO: Calculate the change in radians since the last update.
        rightValue *= -1;

        float delta_left_radians = leftValue - lastLPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEYARE NOT THE WRONG DIRECTION 
        float delta_right_radians = rightValue - lastRPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEY ARE NOT THE WRONG DIRECTION 

        // Serial.println(leftValue);
        // Serial.println(rightValue);

        //TODO: Calculate the foward kinematics
        auto delta_s = WHEEL_RADIUS*delta_left_radians/2.0  + WHEEL_RADIUS*delta_right_radians/2.0;

        x += delta_s*cos(h);
        y += delta_s*sin(h);
        h += -1*WHEEL_RADIUS*delta_left_radians/AXLE_LEN + WHEEL_RADIUS*delta_right_radians/AXLE_LEN;

        lastLPos = leftValue;
        lastRPos = rightValue;
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getH() const { return h; }

private:
    float x, y, h;
    const float R, L;
    float lastLPos, lastRPos;
};

}
