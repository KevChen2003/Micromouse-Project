#pragma once

#include <Arduino.h>

namespace mtrn3100 {
    class EncoderOdometry {
    public:
        EncoderOdometry(float wheel_radius, float axle_length);
        void update(float leftValue, float rightValue);
        float getX() const;
        float getY() const;
        float getH() const;
    private:
        const float wheel_radius = 0.016;
        const float axle_length = 0.1;
        float x, y, h;
        const float R, L;
        float lastLPos, lastRPos;
    };
}

