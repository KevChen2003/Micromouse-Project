// OLED not done
// PID controller not done
// implement the simple movement functions
// hardware: IMU broken, needs checking

#include "robot.hpp"

unsigned long print_timer = 0;

void setup() {
    // Initialization code for components
    Wire.begin();
    Serial.begin(9600);

    lidar.setupLidars(leftLidar_pin, rightLidar_pin, frontLidar_pin);
    imu.setupIMU(mpu);
    // oled.setupOLED(display);

    // l_forward_pid.zeroAndSetTarget(encoder.getLeftRotation(), target);    
    // r_forward_pid.zeroAndSetTarget(-encoder.getRightRotation(), target);
}

void loop() {
    // delay(50);
    // Update encoder odometry
    // encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());

    // moveNCellsForward(5);
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
    // turnRight(4);
    // turnLeft(4);

    moveNCellsForward(4);
    
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

    // mpu.update();
    // if ((millis() - timer) > 1000) {
    //   Serial.print("Angle X: ");
    //   Serial.println(mpu.getAngleX());

    //   Serial.print("Angle Y: ");
    //   Serial.println(mpu.getAngleY());

    //   Serial.print("Angle Z: ");
    //   Serial.println(mpu.getAngleZ());
    //   timer = millis();
    // }

    // Add a delay to control loop rate
    // delay(1500);
};

void moveNCellsForward(int nCells) {

    front_lidar_pid.zeroAndSetTarget(lidar.getFrontLidar(), 80);

    side_lidar_pid.zeroAndSetTarget(getSideLidarError(), 0);

    for (int i = 0; i < nCells; i++) {
        l_forward_pid.zeroAndSetTarget(encoder.getLeftRotation(), 250/16);   
        r_forward_pid.zeroAndSetTarget(-encoder.getRightRotation(), 250/16);
        // the line below means we have to place hte robot near-center otherwise the side lidar error will get zeroed
        while(!moveOneCellForward()){};
        // Serial.print("Count: ");
        // Serial.println(i);
    }
    // delay(1500);
    return false;
}

bool moveOneCellForward() {
    if (lidar.getFrontLidar() < 80) {
        stopMotors();
        return true;
    } 

    float pidL_signal = l_forward_pid.compute(encoder.getLeftRotation());
    float pidR_signal = r_forward_pid.compute(-encoder.getRightRotation());
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

    float l_signal = constrain(- side_lidar_signal + front_lidar_signal, -255, 255);
    float r_signal = constrain(- side_lidar_signal + front_lidar_signal, -255, 255);

    L_Motor.setPWM(l_signal);
    R_Motor.setPWM(-r_signal);

    // L_Motor.setPWM(pidL_signal);
    // R_Motor.setPWM(-pidR_signal);

    // if (abs(pidL_signal) < 12 && abs(pidR_signal < 12)) {
    //     // delay(1500);
    //     return true;
    // } 

    // delay(1500);
    return false;
}

void turnRight(int nTurns) {
    // gives negative H value for turning right
    
    // Serial.print("H: ");
    // Serial.println(encoder_odometry.getH());

    float final_allowed_error = (5 * PI) / 180;
    // float final_allowed_error = (2 * PI) / 180;

    for (int i = 1; i <= nTurns; i++) {
        // encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
        // dont think we need the i here since the current encoder value will be zeroed
        // encoder_odometry_h_pid.zeroAndSetTarget(encoder_odometry.getH(), (-PI/2.0)); 
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
        // float currAngle = mpu.getAngleZ();
        // float setpoint = -90.0;
        // float setpoint = constrain(-90.00,-91.20,-88.80);
        float lower = constrain(-91.00,-91.20,-90.80);
        float higher = constrain(-89.00,-89.20,-88.80);
        float setpoint = constrain(-90.00,lower,higher);
        mpu_pid_right.zeroAndSetTarget(mpu.getAngleZ(),setpoint); 
        // while(!turnRightOnce((float)final_allowed_error / nTurns)){};
        while(!turnRightOnce((float)final_allowed_error)){};
    }
    // Serial.print("H: ");
    // Serial.println(encoder_odometry.getH());
}

bool turnRightOnce(float allowed_error) {

    // encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());

    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

    // float error = encoder_odometry_h_pid.compute(encoder_odometry.getH());
    float error = mpu_pid_right.compute(mpu.getAngleZ());

    // Serial.print("H: ");
    // Serial.println(encoder_odometry.getH());


    if ((millis() - print_timer) > 1000) {
      Serial.print("Error: ");
      Serial.println(error);

      Serial.print("Angle: ");
      Serial.println(mpu.getAngleZ());
      print_timer = millis();
    }

    // float pwm = -error * 5;
    // float pwm = -error;
    float pwm = constrain(-error, -255, 255);

    // if (abs(pwm) < 5) {
    //   pwm *= 6;
    // } else if (abs(pwm) < 10) {
    //   pwm *= 4;
    // }

    // Serial.println(pwm);
    
    R_Motor.setPWM(pwm);
    L_Motor.setPWM(pwm);

    if (abs(error) < allowed_error) {
        // Not going here at all
        // Serial.println(encoder_odometry.getH());
        // stopMotors();
        Serial.println("I'm in the stop loop");
        
        // delay(1500);
        return true;
    }
    // Serial.println("Still turning");
    // delay(1500);
    return false;
}


void turnLeft(int nTurns) {
    // gives negative H value for turning right
    
    // Serial.print("H: ");
    // Serial.println(encoder_odometry.getH());

    float final_allowed_error = (5 * PI) / 180;
    // float final_allowed_error = (2 * PI) / 180;

    for (int i = 1; i <= nTurns; i++) {
        // encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
        // dont think we need the i here since the current encoder value will be zeroed
        // encoder_odometry_h_pid.zeroAndSetTarget(encoder_odometry.getH(), (-PI/2.0)); 
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
        // float currAngle = mpu.getAngleZ();
        // float setpoint = -90.0;
        // float setpoint = constrain(-90.00,-91.20,-88.80);
        float higher = constrain(91.00,90.80,91.20);
        float lower = constrain(89.00,88.80,89.20);
        float setpoint = constrain(90.00,lower,higher);
        mpu_pid_left.zeroAndSetTarget(mpu.getAngleZ(),setpoint); 
        // while(!turnRightOnce((float)final_allowed_error / nTurns)){};
        while(!turnLeftOnce((float)final_allowed_error)){};
    }
    // Serial.print("H: ");
    // Serial.println(encoder_odometry.getH());
}


bool turnLeftOnce(float allowed_error) {

    // encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());

    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

    // float error = encoder_odometry_h_pid.compute(encoder_odometry.getH());
    float error = mpu_pid_left.compute(mpu.getAngleZ());

    // Serial.print("H: ");
    // Serial.println(encoder_odometry.getH());


    if ((millis() - print_timer) > 1000) {
      Serial.print("Error: ");
      Serial.println(error);

      Serial.print("Angle: ");
      Serial.println(mpu.getAngleZ());
      print_timer = millis();
    }

    // float pwm = -error * 5;
    // float pwm = -error;
    float pwm = constrain(-error, -255, 255);

    // if (abs(pwm) < 5) {
    //   pwm *= 6;
    // } else if (abs(pwm) < 10) {
    //   pwm *= 4;
    // }

    // Serial.println(pwm);
    
    R_Motor.setPWM(pwm);
    L_Motor.setPWM(pwm);

    if (abs(error) < allowed_error) {
        // Not going here at all
        // Serial.println(encoder_odometry.getH());
        // stopMotors();
        Serial.println("I'm in the stop loop");
        
        // delay(1500);
        return true;
    }
    // Serial.println("Still turning");
    // delay(1500);
    return false;
}