#define move_forward 1

void moveForward(int cell_count) {

    encoder.l_position = 0;
    encoder.r_position = 0;

    // Set the target position (cellsize - wheel_radius)
    float target = 250 / 23.59 * cell_count;
    
    l_forward_pid.zeroAndSetTarget(encoder.l_position, target);
    r_forward_pid.zeroAndSetTarget(encoder.r_position, target);

    side_lidar_pid.zeroAndSetTarget(0, 0);

    while(!driveMotors(move_forward)) {}; 

    stopMotors();

    delay(500);
}

bool driveMotors(int type) {
    float side_lidar_error = getSideLidarError();

    l_forward_signal = l_forward_pid.compute(encoder.l_position);
    r_forward_signal = r_forward_pid.compute(encoder.r_position);

    side_lidar_signal = side_lidar_pid.compute(side_lidar_error);

    // Serial.print("side: ");
    Serial.println(side_lidar_signal);

    l_drive_signal = l_forward_signal - side_lidar_signal;

    if (l_drive_signal > CONTROL_SPEED) { l_drive_signal = CONTROL_SPEED; }

    L_Motor.setPWM(-l_drive_signal);
    R_Motor.setPWM(r_drive_signal);

    return false;
}
float getSideLidarError() {
    float left_val = lidar.getLeftLidar();

    // float side_lidar_error = 0;
    int no_wall = 100;

    // DIfference between walls
    return left_val - WALL_DETECTION;
}