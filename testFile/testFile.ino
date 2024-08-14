// OLED not done
// straight mocement for 1 cell not working
// chain movement works for left and right

#include "robot.hpp"

unsigned long print_timer = 0;

void setup()
{
    // Initialization code for components
    Wire.begin();
    Serial.begin(9600);
    lidar.setupLidars(leftLidar_pin, rightLidar_pin, frontLidar_pin);
    imu.setupIMU(mpu);
}

void loop()
{
    delay(50);
    // Update encoder odometry
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
    lidar.updateLidars();
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

    // 90 IS TURN LEFT, -90 IS TURN RIGHT
    // char *chain[] = {"f750.00"};
    // char *chain[] = {"f500.00", "t-90.00", "f250", "t-90.00", "f250", "t90", "f250", "t-90", "f1000"};
    // char *chain[] = {"t180.0", "f250.0", "t90.0", "f250.0", "t-90.0", "f250.0", "f250.0", "f250.0", "t90.0", "f250.0", "f250.0", "t90.0", "f250.0", "f250.0", "t-90.0", "f250.0", "t-90.0", "f250.0", "f250.0", "t90.0", "f250.0", "t90.0", "f250.0", "t-90.0", "f250.0", "t90.0", "f250.0", "t90.0", "f250.0", "t-90.0", "f250.0", "t-90.0", "f250.0", "t90.0", "f250.0", "t90.0", "f250.0", "f250.0", "f250.0", "t90.0", "f250.0", "t-90.0", "f250.0", "t-90.0", "f250.0", "t90.0", "f250.0"};

    // char *chain[] = {"t-180", "t90", "t90", "t180", "t-90", "t-90"};
    // char *chain[] = {"f176.7766"};
    // char *chain[] = {"t0.0", "f250.0", "t-90.0", "f250.0", "t0.0", "f250.0", "t0.0", "f250.0", "t90.0", "f250.0", "t0.0", "f250.0", "t0.0", "f250.0", "t-90.0", "f250.0", "t-90.0", "f250.0", "t0.0", "f250.0", "t90.0", "f250.0", "t0.0", "f250.0", "t0.0", "f250.0", "t0.0", "f250.0", "t90.0", "f250.0", "t0.0", "f250.0"};

    // int numElements = sizeof(chain) / sizeof(chain[0]);
    // chainMovements(chain, numElements);

    autonomous();
    // imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
    while (true)
    {
    }
};

void moveDistanceForward(float distance)
{
    front_lidar_pid.zeroAndSetTarget(lidar.getFrontLidar(), 80);
    side_lidar_pid.zeroAndSetTarget(getSideLidarError(), 0);

    float setpoint = distance / 16;

    float target = setpoint + 1.0; // radius
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

    encoder.reset();
    l_forward_pid.zeroAndSetTarget(encoder.getLeftRotation(), target);
    r_forward_pid.zeroAndSetTarget(-encoder.getRightRotation(), target + 0.1);

    while (1)
    {
        lidar.updateLidars();
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

        encoder.readLeftEncoder();
        encoder.readRightEncoder();

        float pidL_signal = l_forward_pid.compute(encoder.getLeftRotation());
        float pidR_signal = r_forward_pid.compute(-encoder.getRightRotation());
        float front_lidar_signal = front_lidar_pid.compute(lidar.getFrontLidar());
        float lidar_left = 0;
        float lidar_right = 0;

        if (abs(l_forward_pid.getError()) < 1.10 && abs(r_forward_pid.getError()) < 1.10)
        {
            stopMotors();
            delay(200);
            break;
        }

        if (lidar.getFrontLidar() <= 90)
        {
            stopMotors();
            break;
        }

        if (lidar.getLeftLidar() <= 75)
        {
            lidar_left = lidar.getLeftLidar();
            pidL_signal = pidL_signal - ((75 / lidar_left) * 20);
        }
        else if (lidar.getRightLidar() <= 75)
        {
            lidar_right = lidar.getRightLidar();
            lidar_right = max(lidar_right, 15);
            pidR_signal = pidR_signal - ((75 / lidar_right) * 20);
        }

        pidL_signal = constrain(pidL_signal, -255, 255);
        pidR_signal = constrain(pidR_signal, -255, 255);

        unsigned long currentMillis = millis();
        if ((currentMillis - prevMillis) > 4000) //5000
        {
            stopMotors();
            break;
        }

        L_Motor.setPWM(pidL_signal * 0.6);
        R_Motor.setPWM(-pidR_signal * 0.6);
    }
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
    delay(100);
}

void moveNCellsForward(int nCells)
{

    front_lidar_pid.zeroAndSetTarget(lidar.getFrontLidar(), 80);
    side_lidar_pid.zeroAndSetTarget(getSideLidarError(), 0);

    int count = 0;
    while (count < nCells)
    {
        float lower = 249.60 / 16;
        float upper = 250.40 / 16;
        float setpoint = 250.00 / 16;
        float target = constrain(setpoint, lower, upper) + 0.8; // radius
        side_lidar_pid.zeroAndSetTarget(getSideLidarError(), 0);
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

        encoder.reset();
        l_forward_pid.zeroAndSetTarget(encoder.getLeftRotation(), target);
        r_forward_pid.zeroAndSetTarget(-encoder.getRightRotation(), target);
        mpu_forward_pid.zeroAndSetTarget(mpu.getAngleZ(), mpu.getAngleZ());
        while (!moveOneCellForward())
        {
        };
        count++;
    }

    delay(1000);
    return false;
}

bool moveOneCellForward()
{
    lidar.updateLidars();
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

    encoder.readLeftEncoder();
    encoder.readRightEncoder();

    float pidL_signal = l_forward_pid.compute(encoder.getLeftRotation());
    float pidR_signal = r_forward_pid.compute(-encoder.getRightRotation());
    float front_lidar_signal = front_lidar_pid.compute(lidar.getFrontLidar());
    float mpu_signal = mpu_forward_pid.compute(mpu.getAngleZ());
    float side_lidar_error = getSideLidarError();

    float side_lidar_signal = side_lidar_pid.compute(side_lidar_error);
    float lidar_left = 0;
    float lidar_right = 0;
    if (abs(side_lidar_pid.getError()) < 0.2)
    {
        side_lidar_signal = 0;
    }

    if (abs(l_forward_pid.getError()) < 1 && abs(r_forward_pid.getError()) < 1)
    {
        stopMotors();
        delay(500);
        return true;
    }

    if (lidar.getFrontLidar() <= 90)
    {
        stopMotors();
        return true;
    }

    if (lidar.getLeftLidar() < 73)
    {
        lidar_left = lidar.getLeftLidar();
        pidL_signal = pidL_signal - ((73 / lidar_left) * 20);
    }
    else if (lidar.getRightLidar() < 73)
    {
        lidar_right = lidar.getRightLidar();
        lidar_right = max(lidar_right, 15);
        pidR_signal = pidR_signal - ((73 / lidar_right) * 20);
    }

    pidL_signal = pidL_signal - (lidar_left * 0.3);
    pidR_signal = pidR_signal - (lidar_right * 0.3);

    pidL_signal = constrain(pidL_signal, -255, 255);
    pidR_signal = constrain(pidR_signal, -255, 255);

    unsigned long currentMillis = millis();
    if ((currentMillis - prevMillis) > 4000) //5000
    {
        stopMotors();
        return true;
    }

    L_Motor.setPWM(pidL_signal);
    R_Motor.setPWM(-pidR_signal);

    // delay(1500);
    return false;
}

void turnRight(int nTurns)
{

    float final_allowed_error = (5 * PI) / 180;

    for (int i = 1; i <= nTurns; i++)
    {
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
        float lower = constrain(-91.00, -91.20, -90.80);
        float higher = constrain(-89.00, -89.20, -88.80);
        float setpoint = constrain(-90.00, lower, higher);
        mpu_pid_right.zeroAndSetTarget(mpu.getAngleZ(), setpoint);
        while (!turnRightOnce((float)final_allowed_error))
        {
        };
    }

    delay(500);
}

bool turnRightOnce(float allowed_error)
{

    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
    float error = mpu_pid_right.compute(mpu.getAngleZ());

    if ((millis() - print_timer) > 1000)
    {
        Serial.print("Error: ");
        Serial.println(error);
        Serial.print("Angle: ");
        Serial.println(mpu.getAngleZ());
        print_timer = millis();
    }
    float pwm = constrain(-error, -255, 255);

    if (abs(mpu_pid_right.getError()) < 0.5)
    {
        stopMotors();
        return true;
    }

    unsigned long currentMillis = millis();
    if ((currentMillis - prevMillis) > 4000) //5000
    {
        stopMotors();
        return true;
    }

    R_Motor.setPWM(pwm);
    L_Motor.setPWM(pwm);

    if (abs(error) < allowed_error)
    {
        Serial.println("I'm in the stop loop");
        return true;
    }
    return false;
}

void turnLeft(int nTurns)
{
    float final_allowed_error = (5 * PI) / 180;

    for (int i = 1; i <= nTurns; i++)
    {
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
        float higher = constrain(91.00, 90.80, 91.20);
        float lower = constrain(89.00, 88.80, 89.20);
        float setpoint = constrain(90.00, lower, higher);
        mpu_pid_left.zeroAndSetTarget(mpu.getAngleZ(), setpoint);
        while (!turnLeftOnce((float)final_allowed_error))
        {
        };
    }
    delay(500);
}

bool turnLeftOnce(float allowed_error)
{
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
    float error = mpu_pid_left.compute(mpu.getAngleZ());

    if ((millis() - print_timer) > 1000)
    {
        Serial.print("Error: ");
        Serial.println(error);
        Serial.print("Angle: ");
        Serial.println(mpu.getAngleZ());
        print_timer = millis();
    }

    if (abs(mpu_pid_left.getError()) < 0.5)
    {
        stopMotors();
        return true;
    }

    float pwm = constrain(-error, -255, 255);

    unsigned long currentMillis = millis();
    if ((currentMillis - prevMillis) > 4000) //5000
    {
        stopMotors();
        return true;
    }

    R_Motor.setPWM(pwm);
    L_Motor.setPWM(pwm);

    if (abs(error) < allowed_error)
    {
        Serial.println("I'm in the stop loop");
        return true;
    }
    return false;
}

void turnNDegrees(float degrees)
{
    float final_allowed_error = (5 * PI) / 180;

    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

    mtrn3100::PIDController pid_turn(0, 0, 0);
    if (degrees > 0)
    {
        pid_turn = mpu_pid_left;
    }
    else if (degrees < 0)
    {
        pid_turn = mpu_pid_right;
    }
    pid_turn.zeroAndSetTarget(mpu.getAngleZ(), degrees);

    while (1)
    {
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
        float error = pid_turn.compute(mpu.getAngleZ());

        if (abs(pid_turn.getError()) < 0.5)
        {
            stopMotors();
            break;
        }

        float pwm = constrain(-error, -255, 255);
        R_Motor.setPWM(pwm * 0.8);
        L_Motor.setPWM(pwm * 0.8);

        if (abs(error) < final_allowed_error)
        {
            break;
        }

        unsigned long currentMillis = millis();
        if ((currentMillis - prevMillis) > 5000)
        {
            stopMotors();
            break;
        }
    };
    delay(100);

}

void autonomous() {
    imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

    while(1) {
        lidar.updateLidars();
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

        Serial.print("L:");
        Serial.println(lidar.getLeftLidar());
        Serial.print("R:");
        Serial.println(lidar.getRightLidar());
        // If no wall is read turn left move forward
        if (lidar.getLeftLidar() > 200 || lidar.getLeftLidar() == 255) {
            Serial.println("HIIIII");
            prevMillis = millis();
            turnNDegrees(90);
            delay(1000);
            prevMillis = millis();
            moveDistanceForward(250.00);
        // If wall is read, move straight
        } else if (lidar.getFrontLidar() > 200) {
            prevMillis = millis();
            moveDistanceForward(250.00);
        
        } else if (lidar.getRightLidar() > 200 || lidar.getRightLidar() == 255) {
            Serial.println("HEYYYYY");
            prevMillis = millis();
            turnNDegrees(-90);
            delay(1000);
            prevMillis = millis();
            moveDistanceForward(250.00);
        } else {
            prevMillis = millis();
            turnNDegrees(-90);
            delay(2000);
            turnNDegrees(-90);
            prevMillis = millis();
            moveDistanceForward(250.00);
        }
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
        delay(50);
    }
    
}

void chainMovements(char *chain[], const int numElements)
{
    for (int i = 0; i < numElements; i++)
    {
        char command = chain[i][0]; // Get the first letter of the command
        float number;
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);

        if (command == 'r')
        {
            turnRight(1);
        }
        else if (command == 'l')
        {
            turnLeft(1);
        }
        else if (command == 'f')
        {
            number = atof(&chain[i][1]); // Convert the string starting from the second character to a float

            int nTimes = (int)(number / 250.00);
            float remaining = number - (250 * nTimes);

            for (int i = 0; i < nTimes; i++)
            {
                prevMillis = millis();
                moveDistanceForward(250.00);
            }
            prevMillis = millis();
            moveDistanceForward(remaining);
        }
        else if (command == 't')
        {   
            number = atof(&chain[i][1]); // Convert the string starting from the second character to a float

            int nTimes = (int)abs(number / 90.00);
            float remaining = abs(number) - (90 * nTimes);
            
            prevMillis = millis();
            turnNDegrees(number);
        }
        imu.updateIMU(mpu, yawReadings, numReadings, index, timer);
        delay(50);
    }
}