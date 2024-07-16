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
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);\
    chainMovement("rrll");  
    // moveNCellsForward(1);  
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
        l_forward_pid.zeroAndSetTarget(encoder.getLeftRotation(), 250/16);    
        r_forward_pid.zeroAndSetTarget(-encoder.getRightRotation(), 250/16);
        while(!moveOneCellForward()){};
        count++;
        Serial.print("Count: ");
        Serial.println(count);
    }

    delay(1500);
    return false;
}

bool moveOneCellForward() {
  if (lidar.getFrontLidar() <= 80) {
        stopMotors();
        return true;
    } 
    float pidL_signal = l_forward_pid.compute(-encoder.getLeftRotation());
    float pidR_signal = r_forward_pid.compute(encoder.getRightRotation());
    float front_lidar_signal = front_lidar_pid.compute(lidar.getFrontLidar());
    float side_lidar_error = getSideLidarError();

    float side_lidar_signal = side_lidar_pid.compute(side_lidar_error);

    if ((millis() - print_timer) > 1000) {
        Serial.print("Left pid: ");
        Serial.println(pidL_signal);

        Serial.print("Right pid: ");
        Serial.println(pidR_signal);
        print_timer = millis();
    }

    // pidL_signal = constrain(pidL_signal, -255, 255);
    // pidR_signal = constrain(pidR_signal, -255, 255);

    // float l_signal = constrain(pidL_signal - side_lidar_signal + front_lidar_signal, -255, 255);
    // float r_signal = constrain(-pidR_signal - side_lidar_signal + front_lidar_signal, -255, 255);

    // float l_signal = constrain(- side_lidar_signal + front_lidar_signal, -255, 255);
    // float r_signal = constrain(- side_lidar_signal + front_lidar_signal, -255, 255);

    // L_Motor.setPWM(l_signal);
    // R_Motor.setPWM(-r_signal);

    L_Motor.setPWM(pidL_signal);
    R_Motor.setPWM(-pidR_signal);

    if (abs(pidL_signal) < 12 && abs(pidR_signal < 12)) {
        // delay(1500);
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
            moveNCellsForward(1); // ????????????? i dunno
        }
        delay(50);
    }
}





