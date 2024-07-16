#define move_forward 1

void moveForward(int cell_count) {

    // encoder.l_position = 0;
    // encoder.r_position = 0;

    // Set the target position (cellsize - wheel_radius)
    float target = (250 / (16 * PI)) * cell_count;
    
    // l_forward_pid.zeroAndSetTarget(encoder.l_position, target);
    // r_forward_pid.zeroAndSetTarget(encoder.r_position, target);
   
    front_lidar_pid.zeroAndSetTarget(lidar.getFrontLidar(), 80);
    side_lidar_pid.zeroAndSetTarget(getSideLidarError(), 0);


    while(!driveMotors(move_forward)) {}; 

    stopMotors();

    delay(500);
}

bool driveMotors(int type) {
    float side_lidar_error = getSideLidarError();

    bool driveWithLidar = false;

    if (driveWithLidar) {
        front_lidar_signal = front_lidar_pid.compute(lidar.getFrontLidar());

        side_lidar_signal = side_lidar_pid.compute(side_lidar_error);

        left_signal = front_lidar_signal + side_lidar_signal;

        right_signal = front_lidar_signal - side_lidar_signal;
    } else {
        // l_forward_signal = l_forward_pid.compute(encoder.l_position);
        // r_forward_signal = r_forward_pid.compute(encoder.r_position);

        side_lidar_signal = side_lidar_pid.compute(side_lidar_error);

        left_signal = l_forward_signal + side_lidar_signal;

        right_signal = -(r_forward_signal - side_lidar_signal);
    }

    // Serial.print("side lidar signal: ");
    // Serial.println(side_lidar_signal);

    // Serial.print("left signal: ");
    // Serial.println(left_signal);

    // Serial.print("right signal: ");
    // Serial.println(right_signal);

    // if (l_drive_signal > CONTROL_SPEED) { l_drive_signal = CONTROL_SPEED; }
    // if (r_drive_signal > CONTROL_SPEED) { r_drive_signal = CONTROL_SPEED; }

    if (lidar.getFrontLidar() <= 80) {
        stopMotors();
        return true;
    }
    
    L_Motor.setPWM(-left_signal); 
    R_Motor.setPWM(right_signal);

    return false;
}
float getSideLidarError() {
    float error = 0;
    float left_val = lidar.getLeftLidar();
    float right_val = lidar.getRightLidar();

    error = left_val-right_val;

    // float side_lidar_error = 0;
    // int no_wall = 100;

    // DIfference between walls
    return -error;
}

void stopMotors(){
  L_Motor.setPWM(0); 
  R_Motor.setPWM(0);
}