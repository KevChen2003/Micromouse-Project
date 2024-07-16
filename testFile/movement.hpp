// #pragma once

#ifndef MOVEMENT_H
#define MOVEMENT_H


// Function declarations for Movement
void moveForward(int cell_count);
int driveMotors(float l_target, float r_target);

mtrn3100::PIDController l_forward_pid(20,10, 0);
mtrn3100::PIDController r_forward_pid(20,10, 0);

mtrn3100::PIDController side_lidar_pid(0,0,0);

float forward_signal = 0;
float side_lidar_signal = 0;
float drive_signal = 0;

const int CONTROL_SPEED = 80;
#endif