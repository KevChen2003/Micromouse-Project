#define move_forward 1

// use THIS for straight line movement using lidars
void moveForwardWithLidar(int cell_count) {
    float target = (250 / 16) * cell_count;

    l_forward_pid.zeroAndSetTarget(encoder.l_position, target);
    r_forward_pid.zeroAndSetTarget(encoder.r_position, target);
   
    front_lidar_pid.zeroAndSetTarget(lidar.getFrontLidar(), 100);
    side_lidar_pid.zeroAndSetTarget(getSideLidarError(), 0);


    while(!driveMotors(move_forward)) {}; 
}

bool driveMotors(int type) {
    float side_lidar_error = getSideLidarError();

    bool driveWithLidar = true;

    if (driveWithLidar) {
        lidar.updateLidars();
        front_lidar_signal = front_lidar_pid.compute(lidar.getFrontLidar());

        side_lidar_signal = side_lidar_pid.compute(side_lidar_error);

        left_signal = (front_lidar_signal + side_lidar_signal);

        right_signal = (front_lidar_signal - side_lidar_signal);
    }

    if (lidar.getFrontLidar() <= 100) {
        stopMotors();
        return true;
    }
    
    L_Motor.setPWM(left_signal); 
    R_Motor.setPWM(-right_signal);

    return false;
}
float getSideLidarError() {
    float error = 0;
    float left_val = lidar.getLeftLidar();
    float right_val = lidar.getRightLidar();

    error = left_val-right_val;
    return error;
}

void stopMotors(){
  L_Motor.setPWM(0); 
  R_Motor.setPWM(0);
}


// void moveForward(int cellCount) {
//     const float distancePerCell = 250.0; // in mm
//     const float targetDistance = cellCount * distancePerCell; // in mm
//     const float targetCounts = targetDistance / distance_per_count; // in encoder counts

//     float initialLeftCounts = encoder.getLeftRotation();
//     float initialRightCounts = encoder.getRightRotation();

//     float distanceTravelledCounts = 0;
    
//     // Move forward until the target distance is reached
//     while (distanceTravelledCounts < targetCounts) {
//         // Update the encoder readings
//         encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
        
//         // Calculate the distance travelled in counts
//         float currentLeftCounts = encoder.getLeftRotation();
//         float currentRightCounts = encoder.getRightRotation();
//         distanceTravelledCounts = ((currentLeftCounts - initialLeftCounts) + (currentRightCounts - initialRightCounts)) / 2.0;

//         // Set motor speeds for forward movement
//         L_Motor.setPWM(-80); // Adjust PWM values as needed
//         R_Motor.setPWM(80);

//         // Add a small delay for the loop
//         delay(10);
//     }

//     // Stop the motors once the target distance is reached
//     stopMotors();
// }






