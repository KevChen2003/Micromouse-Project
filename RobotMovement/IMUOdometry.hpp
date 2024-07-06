#ifndef IMU_ODOMETRY_HPP
#define IMU_ODOMETRY_HPP

#include <Arduino.h>
#include <MPU6050_light.h>

namespace mtrn3100 {
    class IMUOdometry {
    public:
        IMUOdometry() : x(0), y(0), vx(0), vy(0), yaw(0), lastUpdateTime(millis()) {}

        void update(float accel_x, float accel_y) {
            unsigned long currentTime = millis();
            float dt = (currentTime - lastUpdateTime);  // Convert to seconds
            lastUpdateTime = currentTime;

            // Integrate acceleration to get velocity
            vx += accel_x * dt/1000;
            vy += accel_y * dt/1000;

            yaw = imu.getAngleZ();

            // TODO: Integrate velocity to get position
            x += vx * dt/1000;
            y += vy * dt/1000;
        }

        float getX() const { return x; }
        float getY() const { return y; }
        float getYaw() const { return yaw; }

    private:
        float x, y;
        float vx, vy;
        float yaw;
        unsigned long lastUpdateTime;
        MPU6050_light imu;
    };
}

#endif // IMU_ODOMETRY_HPP
