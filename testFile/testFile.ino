// OLED not done
// straight mocement for 1 cell not working
// chain movement works for left and right

#include "robot.hpp"

unsigned long print_timer = 0;

void setup() {
    // Initialization code for components
    Wire.begin();
    Serial.begin(9600);
    lidar.setupLidars(leftLidar_pin, rightLidar_pin, frontLidar_pin);
    imu.setupIMU(mpu);
}

void loop() {
    delay(50);
    // Update encoder odometry
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
    lidar.updateLidars();    
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

    // moveForwardWithLidar(5);
    chainMovement("ffrrfrfl");
    // chainMovement("rrrr");


    // moveNCellsForward(6);
    // moveForwardWithLidar(6);
    // lidar.updateLidars();
    // imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
    while(true){}
};

void moveNCellsForward(int nCells) {
    
    front_lidar_pid.zeroAndSetTarget(lidar.getFrontLidar(), 80);
    side_lidar_pid.zeroAndSetTarget(getSideLidarError(), 0);

    // for (int i = 0; i < nCells; i++) {
    //     l_forward_pid.zeroAndSetTarget(encoder.getLeftRotation(), 250/16);    
    //     r_forward_pid.zeroAndSetTarget(encoder.getRightRotation(), 250/16);
    //     while(!moveOneCellForward()){};
    //     Serial.print("Count: ");
    //     Serial.println(i);
    // }
    int count = 0;
    while (count < nCells) {
      float lower = 249.60/16;
      float upper = 250.40/16;
      float setpoint = 250.00/16;
      float target = constrain(setpoint, lower, upper); // radius
        side_lidar_pid.zeroAndSetTarget(getSideLidarError(), 0);

        encoder.reset();
        l_forward_pid.zeroAndSetTarget(encoder.getLeftRotation(), target);    
        r_forward_pid.zeroAndSetTarget(-encoder.getRightRotation(), target);
        while(!moveOneCellForward()){};
        count++;
    }

    delay(1000);
    return false;
}

bool moveOneCellForward() {
    lidar.updateLidars();

    encoder.readLeftEncoder();
    encoder.readRightEncoder();

    float pidL_signal = l_forward_pid.compute(encoder.getLeftRotation());
    float pidR_signal = r_forward_pid.compute(-encoder.getRightRotation());
    float front_lidar_signal = front_lidar_pid.compute(lidar.getFrontLidar());
    float side_lidar_error = getSideLidarError();

    float side_lidar_signal = side_lidar_pid.compute(side_lidar_error);

    if (abs(side_lidar_pid.getError()) < 0.2) {
      side_lidar_signal = 0;
    }

    // // Stop if the encoder error is small enough
    // if (abs(abs(l_forward_pid.getError()) - abs(side_lidar_pid.getError())) < 0.25 
    //   && abs(abs(r_forward_pid.getError()) - abs(side_lidar_pid.getError())) < 0.25) {
    //     stopMotors();
    //     delay(500);
    //     return true;
    // }

    if (abs(l_forward_pid.getError()) < 0.5 && abs(r_forward_pid.getError()) < 0.5) {
        stopMotors();
        delay(500);
        return true;
    }

    // pidL_signal = pidL_signal + (side_lidar_signal * 1.04);
    // pidR_signal = pidR_signal - (side_lidar_signal * 1.04);

    pidL_signal = constrain(pidL_signal, -255,255);
    pidR_signal = constrain(pidR_signal, -255,255);

    L_Motor.setPWM(pidL_signal);
    R_Motor.setPWM(-pidR_signal);

    if (lidar.getFrontLidar() <= 100) {
        stopMotors();
        return true;
    }

    // delay(1500);
    return false;
}


void turnRight(int nTurns) {

    float final_allowed_error = (5 * PI) / 180;

    for (int i = 1; i <= nTurns; i++) {
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
        float lower = constrain(-91.00,-91.20,-90.80);
        float higher = constrain(-89.00,-89.20,-88.80);
        float setpoint = constrain(-90.00,lower,higher);
        mpu_pid_right.zeroAndSetTarget(mpu.getAngleZ(),setpoint); 
        while(!turnRightOnce((float)final_allowed_error)){};
    }
}

bool turnRightOnce(float allowed_error) {

    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
    float error = mpu_pid_right.compute(mpu.getAngleZ());

    if ((millis() - print_timer) > 1000) {
      Serial.print("Error: ");
      Serial.println(error);
      Serial.print("Angle: ");
      Serial.println(mpu.getAngleZ());
      print_timer = millis();
    }
    float pwm = constrain(-error, -255, 255);
    
    if (abs(mpu_pid_right.getError()) < 0.5) {
        stopMotors();
        return true;
    }

    R_Motor.setPWM(pwm);
    L_Motor.setPWM(pwm);

    if (abs(error) < allowed_error) {
        Serial.println("I'm in the stop loop");
        return true;
    }
    return false;
}


void turnLeft(int nTurns) {
    float final_allowed_error = (5 * PI) / 180;

    for (int i = 1; i <= nTurns; i++) {
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
        float higher = constrain(91.00,90.80,91.20);
        float lower = constrain(89.00,88.80,89.20);
        float setpoint = constrain(90.00,lower,higher);
        mpu_pid_left.zeroAndSetTarget(mpu.getAngleZ(),setpoint); 
        while(!turnLeftOnce((float)final_allowed_error)){};
    }
}


bool turnLeftOnce(float allowed_error) {
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
    float error = mpu_pid_left.compute(mpu.getAngleZ());

    if ((millis() - print_timer) > 1000) {
      Serial.print("Error: ");
      Serial.println(error);
      Serial.print("Angle: ");
      Serial.println(mpu.getAngleZ());
      print_timer = millis();
    }

    if (abs(mpu_pid_left.getError()) < 0.5) {
        stopMotors();
        return true;
    }

    float pwm = constrain(-error, -255, 255);    
    R_Motor.setPWM(pwm);
    L_Motor.setPWM(pwm);

    if (abs(error) < allowed_error) {
        Serial.println("I'm in the stop loop");
        return true;
    }
    return false;
}


void chainMovement(const char* sequence) {
    for (int i = 0; sequence[i] != '\0'; i++) {
        char move = sequence[i];
        if (move == 'r' || move == 'R') {
            turnRight(1);
        } else if (move == 'l' || move == 'L') {
            turnLeft(1); 
        } else if (move == 'f' || move == 'F') {
            moveNCellsForward(1); 
        }
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
        delay(50);
    }
}





