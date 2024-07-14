#include "EncoderOdometry.hpp"

mtrn3100::EncoderOdometry::EncoderOdometry(float wheel_radius, float axle_length) 
 : wheel_radius(wheel_radius), axle_length(axle_length), x(0), y(0), h(0), R(wheel_radius), L(axle_length), lastLPos(0), lastRPos(0) {}

void mtrn3100::EncoderOdometry::update(float leftValue, float rightValue) {
    rightValue *= -1; //  Encoders face each other so one must rotate in the opposite direction to move forward

    float delta_left_radians = leftValue - lastLPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEY ARE NOT THE WRONG DIRECTION 
    float delta_right_radians = rightValue - lastRPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEY ARE NOT THE WRONG DIRECTION 

    // Serial.println(leftValue);
    // Serial.println(rightValue);

    // Forward kinematics formula
    auto delta_s = wheel_radius*delta_left_radians/2.0  + wheel_radius*delta_right_radians/2.0;

    x += delta_s*cos(h);
    y += delta_s*sin(h);
    h += -1*wheel_radius*delta_left_radians/axle_length + wheel_radius*delta_right_radians/axle_length;

    lastLPos = leftValue;
    lastRPos = rightValue;
}

float mtrn3100::EncoderOdometry::getX() const { return x; }
float mtrn3100::EncoderOdometry::getY() const { return y; }
float mtrn3100::EncoderOdometry::getH() const { return h; }
