#include "MicroMouse.h"

MicroMouse Runner;

MicroMouse::MicroMouse(float temp)
    : first_turn(false), rightWallFollow(false), leftWallFollow(false), temperatureCentigrade(temp)
{
    Wire.setSCL(SCL);
    Wire.setSDA(SDA);
    Wire.setClock(WIRE_SPEED);
    Wire.begin();
}

MicroMouse::~MicroMouse()
{
}

void MicroMouse::init()
{
    SHTC3_Status_TypeDef tempStatus = MicroMouse::mySHTC3.begin();
    errorDecoder(tempStatus);
    // Serial.printf("Update Temp Succeed: &d\n", (int)tempStatus);
    if (tempStatus == SHTC3_Status_Nominal)
    {
        updateTempC();
    }
    humidity = MicroMouse::mySHTC3.toPercent();
    soundsp = 331.4 + (0.606 * temperatureCentigrade) + (0.0124 * humidity);
    soundcm = soundsp / 10000.0;
    pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);
    // Input = sensorArray[Left].ping_median();
    Setpoint = 7;
    myPID.SetMode(myPID.Control::automatic);
    myPID.SetOutputLimits(-15, 15);
}

void MicroMouse::setTempC(float temp)
{
    temperatureCentigrade = temp;
}

float MicroMouse::getTempC() const
{
    return temperatureCentigrade;
}

void MicroMouse::updateTempC()
{
    // Add if to prevent update errors.
    SHTC3_Status_TypeDef result = MicroMouse::mySHTC3.update();
    temperatureCentigrade = MicroMouse::mySHTC3.toDegC();
}

void MicroMouse::setDirection(int dir)
{
    Serial.print(F("RMS: "));
    Serial.println(RMS);
    Serial.print(F("LMS: "));
    Serial.println(LMS);
    Serial.print(F("Direction: "));
    Serial.println(dir);
    if (dir == FORWARD)
    {
        // RobotCarPWMMotorControl.goDistanceMillimeter(100);
        // delay(400);
        // RobotCarPWMMotorControl.leftCarMotor.setSpeedPWM(LMS, DIRECTION_FORWARD);
        // RobotCarPWMMotorControl.rightCarMotor.setSpeedPWM(RMS, DIRECTION_FORWARD);
        // RobotCarPWMMotorControl.leftCarMotor.goDistanceMillimeter();
        // RobotCarPWMMotorControl.rightCarMotor.goDistanceMillimeter(RMS,DIRECTION_FORWARD);

        digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH); // Left wheel forward
        digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
        digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH); // Right wheel forward
        digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
        Serial.println("FORWARD");
    }
    else if (dir == LEFT)
    {
        /*         RobotCarPWMMotorControl.leftCarMotor.setSpeedPWM(LMS, DIRECTION_BACKWARD);
                RobotCarPWMMotorControl.rightCarMotor.setSpeedPWM(RMS, DIRECTION_FORWARD); */
        digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW); // Left wheel reverse
        digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
        digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH); // Right wheel forward
        digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
        Serial.println("LEFT");
    }
    else if (dir == RIGHT)
    {
        /*         RobotCarPWMMotorControl.leftCarMotor.setSpeedPWM(LMS, DIRECTION_FORWARD);
                RobotCarPWMMotorControl.rightCarMotor.setSpeedPWM(RMS, DIRECTION_BACKWARD); */
        digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH); // Left wheel forward
        digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
        digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW); // Right wheel reverse
        digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
        Serial.println("RIGHT");
    }
    else if (dir == STOP)
    {
        /*         RobotCarPWMMotorControl.leftCarMotor.stop(MOTOR_BRAKE);
                RobotCarPWMMotorControl.rightCarMotor.stop(MOTOR_BRAKE); */
        digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH); // Left wheel stop
        digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
        digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH); // Right wheel stop
        digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
        Serial.println("STOP");
    }
    else if (dir == BACKWARD)
    {
        /*        RobotCarPWMMotorControl.leftCarMotor.setSpeedPWM(LMS, DIRECTION_BACKWARD);
               RobotCarPWMMotorControl.rightCarMotor.setSpeedPWM(RMS, DIRECTION_BACKWARD); */
        digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH); // Left wheel reverse
        digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
        digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH); // Right wheel backward
        digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
        Serial.println("BACKWARD");
    }
}

void MicroMouse::ReadSensors()
{

    /*     // leftSensor = sonarLeft.ping_median(TestNUM);     //accurate but slow
        // rightSensor = sonarRight.ping_median(TestNUM);     //accurate but slow
        // frontSensor = sonarFront.ping_median(TestNUM);     //accurate but slow

        // leftSensor = sonarLeft.convert_cm(leftSensor);
        // rightSensor = sonarRight.convert_cm(rightSensor);
        // frontSensor = sonarFront.convert_cm(frontSensor);

        // Sensor[Left][New] = sensorArray[Left].measureDistanceCm(temperatureCentigrade); // ping in cm
        // Sensor[Right][New] = sensorArray[Front].measureDistanceCm(temperatureCentigrade);
        // Sensor[Front][New] = sensorArray[Right].measureDistanceCm(temperatureCentigrade); */

    // Prehaps combine this into one for loop if the timing allows.
    // SensorPing[Left] = sensorArray[Left].ping_median(5, temperatureCentigrade);
    // SensorPing[Right] = sensorArray[Right].ping_median(5, temperatureCentigrade);

    for (uint8_t i = 0; i < 3; i++)
    {
        // This may possibly work. I just don't like that the robot is in motion while it is performing these calculations.
        Sensor[i][Old] = Sensor[i][Average] = (sensorArray[i].ping_cm(MAX_DISTANCE) + Sensor[i][Old]) / 2.0;
        // SensorPing[i] = sensorArray[i].ping_median(5);
        // Sensor[i][Average] = (SensorPing[i] / 2.0) * soundcm - 2.0;
        // delay(12);
        //  Sensor[i][Average] = sensorArray[i].ping_medianCM(10, temperatureCentigrade);
        //   delay(30);
        //    Sensor[i][New] = sensorArray[i].measureDistanceCm(temperatureCentigrade);
    }
    // for (uint8_t i = 0; i < 3; i++)
    /*     {
            Sensor[i][Old] = Sensor[i][Average] = (Sensor[i][New] + Sensor[i][Old]) / 2;
            // Sensor[i][Old] = Sensor[i][Average];
        } */

    // lSensor = sonarLeft.measureDistanceCm(temperatureCentigrade); // ping in cm
    // rSensor = sonarRight.measureDistanceCm(temperatureCentigrade);
    // fSensor = sonarFront.measureDistanceCm(temperatureCentigrade);

    // Sensor[Left][Average] = (Sensor[Left][New] + Sensor[Left][Old]) / 2;    // average distance between old & new readings to make the change smoother
    // Sensor[Right][Average] = (Sensor[Right][New] + Sensor[Right][Old]) / 2; // It will truncate by division, maybe that is needed.
    // Sensor[Front][Average] = (Sensor[Front][New] + Sensor[Front][Old]) / 2;

    // Sensor[Left][Old] = Sensor[Left][Average]; // save old readings for movment
    // Sensor[Right][Old] = Sensor[Right][Average];
    // Sensor[Front][Old] = Sensor[Front][Average];
}

// void MicroMouse::pid_start()
// {

//     // ReadSensors()

//     float errorP = Sensor[Left][Average] - Sensor[Right][Average];
//     float errorD = errorP - oldErrorP;
//     float errorI = (2.0 / 3.0) * errorI + errorP;
//     totalError = P * errorP + D * errorD + I * errorI;
//     oldErrorP = errorP;

//         RMS = baseSpeed + totalError;
//         LMS = baseSpeed - totalError;

//     //  if(RMS < -255) RMS = -255; if(RMS > 255)RMS = 255 ;
//     //  if(LMS < -255) LMS = -255;  if(LMS > 255)LMS = 255 ;

//     if (RMS < 0)
//     {

//         RMS = map(RMS, 0, -255, 0, 255);

//         // analogWrite(RIGHT_MOTOR_PWM_PIN, RMS);
//         // analogWrite(LEFT_MOTOR_PWM_PIN, LMS);

//         setDirection(RIGHT);
//     }
//     else if (LMS < 0)
//     {
//         LMS = map(LMS, 0, -255, 0, 255);

//         // analogWrite(RIGHT_MOTOR_PWM_PIN, RMS);
//         // analogWrite(LEFT_MOTOR_PWM_PIN, LMS);

//         setDirection(LEFT);
//     }
//     else
//     {

//         // analogWrite(RIGHT_MOTOR_PWM_PIN, RMS);
//         // analogWrite(LEFT_MOTOR_PWM_PIN, LMS);

//         setDirection(FORWARD);
//     }
// }

void MicroMouse::PIDcalculate(boolean left)
{
    // Rearrange the checks to reduce the lines of code. If first turn is false, subtract zero, else return a positive offset if left is true, and a negative offset if left is false.

    // Serial.print("first_turn: ");
    // Serial.println(first_turn);
    // Serial.print("Left: ");
    // Serial.println(left);
    // Serial.print("Offset: ");
    // Serial.println(offset);
    // long errorP = Sensor[Left][Average] - Sensor[Right][Average]; // - (first_turn ? (left ? offset : -offset) : 0);
    // errorP = SensorPing[Left] - SensorPing[Right];
    // Serial.print("errorP: ");
    // Serial.println(errorP);
    // float errorD = errorP - oldErrorP;
    // Serial.print("errorD: ");
    // Serial.println(errorD);
    // errorI = (2.0 / 3.0) * errorI + errorP;
    // Serial.print("errorI: ");
    // Serial.println(errorI);
    // totalError = P2 * errorP + /*D * errorD + */ I2 * errorI;
    // oldErrorP = errorP;

    // if (left == true)
    //{

    /*         // float errorP = leftSensor - rightSensor - offset;
            // float errorD = errorP - oldErrorP;
            // float errorI = (2.0 / 3) * errorI + errorP;

            // totalError = P * errorP + D * errorD + I * errorI;

            // oldErrorP = errorP; */
    while (Sensor[Left][Average] < 3 || Sensor[Left][Average] > 15)
    {
        SensorPing[Left] = sensorArray[Left].ping_median(5);
        Sensor[Left][Average] = (SensorPing[Left] / 2.0) * soundcm;
        delay(12);
        setDirection(STOP);
    }
    // while(Sensor[Left][Average] > 10)
    {
        // Sensor[Left][Average] = 0.0;
    }
    Input = Sensor[Left][Average]- 2;

    double gap = abs(Setpoint - Input); // distance away from setpoint
    if (gap < 3.5)
    { // we're close to setpoint, use conservative tuning parameters
        myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
        // we're far from setpoint, use aggressive tuning parameters
        myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    myPID.Compute();
    RMS = baseSpeed[Right] - Output;
    LMS = baseSpeed[Left] + Output;
    Serial.print("Output: ");
    Serial.println(Output);

    /* //  if(RMS < -255) RMS = -255; if(RMS > 255)RMS = 255 ;
    //  if(LMS < -255) LMS = -255;  if(LMS > 255)LMS = 255 ; */

    // if (RMS < 0)
    // {

    //     RMS = map(RMS, 0, -255, 0, 255);

    //     analogWrite(RIGHT_MOTOR_PWM_PIN, RMS);
    //     analogWrite(LEFT_MOTOR_PWM_PIN, LMS);

    //     setDirection(RIGHT);
    // }
    // else if (LMS < 0)
    // {
    //     LMS = map(LMS, 0, -255, 0, 255);

    //     analogWrite(RIGHT_MOTOR_PWM_PIN, RMS);
    //     analogWrite(LEFT_MOTOR_PWM_PIN, LMS);

    //     setDirection(LEFT);
    // }
    // else
    analogWrite(RIGHT_MOTOR_PWM_PIN, RMS);
    analogWrite(LEFT_MOTOR_PWM_PIN, LMS);
    if (false)
    {
        if (Sensor[Left][Average] <= 4)
        {
            // digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH); // Left wheel reverse
            // digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
            // digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW); // Right wheel forward
            // digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
            setDirection(RIGHT);
            delay(10);
            // setDirection(STOP);
            // delay(100);
            // Sensor[Left][Old] = Sensor[Left][Average] = (sensorArray[Left].measureDistanceCm(temperatureCentigrade) + Sensor[Left][Old]) / 2;
        }
        // else if ((Sensor[Left][Average] >= 10 || Sensor[Left][Average] < 2) && false)
        // {
        //     digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW); // Left wheel off
        //     digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
        //     digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH); // Right wheel forward
        //     digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
        //     //  This is where I need to implement the 180 degree rotation using the MPU-6050.
        //     // setDirection(STOP);
        //     // setDirection(STOP);
        //     // analogWrite(RIGHT_MOTOR_PWM_PIN, RMS);
        //     // analogWrite(LEFT_MOTOR_PWM_PIN, LMS);
        //     //setDirection(LEFT);
        //     //delay(100);
        //     // setDirection(STOP);
        //     //delay(50);
        //     // Sensor[Left][Old] = Sensor[Left][Average] = (sensorArray[Left].measureDistanceCm(temperatureCentigrade) + Sensor[Left][Old]) / 2;
        // }
    }
    // if (Sensor[Left][Average] > 5 && Sensor[Left][Average] < 10)
    {

        setDirection(FORWARD);
        // delay(100);
    }
    // while (Sensor[Left][Average] < 2)
    // {

    //     analogWrite(RIGHT_MOTOR_PWM_PIN, RMS);
    //     analogWrite(LEFT_MOTOR_PWM_PIN, LMS);
    //     setDirection(STOP);
    //     Sensor[Left][Average] = sensorArray[left].ping_medianCM(5, temperatureCentigrade);
    // }

    //}
    // /* else
    // {

    //     /*         // float errorP = leftSensor - rightSensor + offset;
    //             // float errorD = errorP - oldErrorP;
    //             // float errorI = (2.0 / 3) * errorI + errorP;

    //             // totalError = P * errorP + D * errorD + I * errorI;

    //             // oldErrorP = errorP;

    //             // RMS = baseSpeed + totalError;
    //             // LMS = baseSpeed - totalError;

    //             //  if(RMS < -255) RMS = -255; if(RMS > 255)RMS = 255 ;
    //             //  if(LMS < -255) LMS = -255;  if(LMS > 255)LMS = 255 ; */

    //     if (RMS < 0)
    //     {

    //         RMS = map(RMS, 0, -255, 0, 255);

    //         // analogWrite(RIGHT_MOTOR_PWM_PIN, RMS);
    //         // analogWrite(LEFT_MOTOR_PWM_PIN, LMS);

    //         setDirection(RIGHT);
    //     }
    //     else if (LMS < 0)
    //     {
    //         LMS = map(LMS, 0, -255, 0, 255);

    //         // analogWrite(RIGHT_MOTOR_PWM_PIN, RMS);
    //         // analogWrite(LEFT_MOTOR_PWM_PIN, LMS);

    //         setDirection(LEFT);
    //     }
    //     else
    //     {

    //         // analogWrite(RIGHT_MOTOR_PWM_PIN, RMS);
    //         // analogWrite(LEFT_MOTOR_PWM_PIN, LMS);

    //         setDirection(FORWARD);
    //     }
    // } */
}

void MicroMouse::walls()
{

    // Just assign the boolean result of the test directly to the variable.
    wall[Left] = Sensor[Left][Average] < wall_threshold;
    /*     if (Sensor[Left][Average] < wall_threshold)
        {
            leftwall = true;
        }
        else
        {
            leftwall = false;
        } */

    wall[Right] = Sensor[Right][Average] < wall_threshold;

    /*     if (Sensor[Right][Average] < wall_threshold)
        {
            rightwall = true;
        }
        else
        {
            rightwall = false;
        } */
    wall[Front] = Sensor[Front][Average] < front_threshold;

    /*     if (Sensor[Front][Average] < front_threshold)
        {
            frontwall = true;
        }
        else
        {
            frontwall = false;
        } */
}

void MicroMouse::turnright()
{

    /*     LMS = baseSpeed; */

    RMS = LMS * Sensor[Right][Average] / (Sensor[Right][Average] + 11);
}

void MicroMouse::turnleft()
{
    /*
        RMS = baseSpeed; */

    LMS = RMS * Sensor[Left][Average] / (Sensor[Left][Average] + 11);
}

// This is a wall hugging mode.
void MicroMouse::runInLoop()
{

    //========================================START========================================//

    ReadSensors();

    walls();

    /*     if (first_turn == false)
        {
            pid_start();
        }
        else */
    if (leftWallFollow || rightWallFollow || !first_turn)
    {
        // If leftWallFollow or rightWallFollow are true, then juse use the boolean of leftWallFollow as the argument.
        PIDcalculate(leftWallFollow);
    }
    /*     else if (leftWallFollow == true)
        {
            PID(true);
        }
        else if (rightWallFollow == true)
        {
            PID(false);
        } */

    if (wall[Front])
    {
        if (wall[Left] != wall[Right])
        {
            PIDcalculate(!wall[Left]);
            if (!first_turn)
            {

                //      left_threshold = left_threshold - offset ;
                //      right_threshold = right_threshold + offset ;

                first_turn = true;
                // If Left is false and Right is true, LeftWallFollow becomes true, else stays the same. Likewise for the RightwallFollow.
                leftWallFollow = (!wall[Left] ? true : leftWallFollow);
                rightWallFollow = (wall[Left] ? true : rightWallFollow);

                // digitalWrite(LED, HIGH);
            }
        }
    }
    /*     if (wall[Left] == true && wall[Right] == false && wall[Front] == true)
        {
            // turnright();
            PID(false);

            if (first_turn == false)
            {

                //      right_threshold = right_threshold - offset ;
                //      left_threshold = left_threshold + offset ;

                first_turn = true;
                rightWallFollow = true;

                // digitalWrite(led2, LOW);
                // digitalWrite(led1, HIGH);
            }
        }
        if (wall[Left] == false && wall[Right] == true && wall[Front] == true)
        {

            //  turnleft();
            PID(true);

            if (first_turn == false)
            {

                //      left_threshold = left_threshold - offset ;
                //      right_threshold = right_threshold + offset ;

                first_turn = true;
                leftWallFollow = true;
                // digitalWrite(LED, HIGH);
            }
        } */
    // if (Sensor[Left][Average] <= 3 || Sensor[Left][Average] > 100 && Sensor[Right][Average] <= 3 || Sensor[Right][Average] > 100 && Sensor[Front][Average] <= 3 || Sensor[Front][Average] > 100)
    //{
    // Serial.println("Stop");
    //  This is where I need to implement the 180 degree rotation using the MPU-6050.
    // setDirection(STOP);
    //}

    // read sensors & print the result to the serial monitor //

    Serial.print(" Left : ");
    Serial.print(Sensor[Left][Average]);
    Serial.print(" cm ");
    Serial.print(" Right : ");
    Serial.print(Sensor[Right][Average]);
    Serial.print(" cm ");
    Serial.print(" Front : ");
    Serial.print(Sensor[Front][Average]);
    Serial.println(" cm ");

    // measure error & print the result to the serial monitor

    Serial.print("error=");
    Serial.println(totalError);
}

void MicroMouse::errorDecoder(SHTC3_Status_TypeDef message) // The errorDecoder function prints "SHTC3_Status_TypeDef" resultsin a human-friendly way
{
    switch (message)
    {
    case SHTC3_Status_Nominal:
        Serial.print("Nominal");
        break;
    case SHTC3_Status_Error:
        Serial.print("Error");
        break;
    case SHTC3_Status_CRC_Fail:
        Serial.print("CRC Fail");
        break;
    default:
        Serial.print("Unknown return code");
        break;
    }
}

// Begin floodfill algorithim

byte MicroMouse::readCurrent()
{
    byte wallReading = 15;
    byte north = 0;
    byte south = 0;
    byte east = 0;
    byte west = 0;
    switch (globalHeading)
    {
    case 1:
        // if the forward sensor is tripped
        if (!wall[Front])
        {
            // set north to 1
            north = 1;
        }
        // if the right sensor is tripped
        if (!wall[Right])
        {
            // set east to 4
            east = 4;
        }
        // if the left sensor is tripped
        if (!wall[Left])
        {
            // set west to 9
            west = 8;
        }
        // Subtract the sum of north east and west from the value of wall reading
        wallReading -= (north + east + west);
        break;
    case 2:
        // if the forward sensor is tripped
        if (!wall[Front])
        {
            // set south to 2
            south = 2;
        }
        // if the right sensor is tripped
        if (!wall[Right])
        {
            // set west to 8
            west = 8;
        }
        // if the left sensor is tripped
        if (!wall[Left])
        {
            // set east to 4
            east = 4;
        }
        // subtract the sum from 15
        wallReading -= (south + east + west);
        break;
    case 4:
        // if the forward sensor is tripped
        if (!wall[Front])
        {
            // set east to 4
            east = 4;
        }
        // if the right sensor is tripped
        if (!wall[Right])
        {
            // set south to 2
            south = 2;
        }
        // if the left sensor is tripped
        if (!wall[Left])
        {
            // set north to 1
            north = 1;
        }
        // subtract the sum from 15
        wallReading -= (north + south + east);
        break;
    case 8:
        // if the forward sensor is tripped
        if (!wall[Front])
        {
            // set east to 8
            west = 8;
        }
        // if the right sensor is tripped
        if (!wall[Right])
        {
            // set north to 1
            north = 1;
        }
        // if the left sensor is tripped
        if (!wall[Left])
        {
            // set south to 1
            south = 2;
        }
        // subtract the sum from 15
        wallReading -= (west + north + south);
        break;
    }
    return wallReading;
}

MicroMouse::instruction MicroMouse::createInstruction(coord currCoord, coord nextCoord, byte nextHeading)
{
    float change = 0.0;
    switch (nextHeading)
    {
    case 1:
        if (globalHeading == 4)
        {
            change = -90.0;
        }
        if (globalHeading == 8)
        {
            change = 90.0;
        }
        if (globalHeading == 2)
        {
            change = 180.0;
        }
        break;
    case 2:
        if (globalHeading == 4)
        {
            change = 90.0;
        }
        if (globalHeading == 8)
        {
            change = -90.0;
        }
        if (globalHeading == 1)
        {
            change = 180.0;
        }
        break;
    case 4:
        if (globalHeading == 1)
        {
            change = 90.0;
        }
        if (globalHeading == 2)
        {
            change = -90.0;
        }
        if (globalHeading == 8)
        {
            change = 180.0;
        }
        break;
    case 8:
        if (globalHeading == 1)
        {
            change = -90.0;
        }
        if (globalHeading == 2)
        {
            change = 90.0;
        }
        if (globalHeading == 4)
        {
            change = 180.0;
        }
        break;
    }
    // Adjust this to give turn instructions.
    float desiredHeading = /* dispatch.gyroVal */ +change;
    // fix over or underflow

    if (((desiredHeading < 45.0) || (desiredHeading > 315.0)))
    {
        // Forward
        desiredHeading = 0.0;
    }
    if ((desiredHeading > 45.0) && (desiredHeading < 135.0))
    {
        // Right
        desiredHeading = 90.0;
    }
    if ((desiredHeading > 135.0) && (desiredHeading < 225.0))
    {
        // Turn a round
        desiredHeading = 180.0;
    }
    if ((desiredHeading > 225.0) && (desiredHeading < 315.0))
    {
        // Left
        desiredHeading = 270.0;
    }

    instruction turnMove = {7.74, desiredHeading};
    return turnMove;
}

// Instantiate the reflood maze with the most optimistic values
void MicroMouse::instantiateReflood()
{
    for (int j = 0; j < Y; j++)
    {
        for (int i = 0; i < X; i++)
        {
            maze[j][i].distance = calcCenter(i, j, X);
        }
    }
}

void MicroMouse::instantiate()
{
    for (int j = 0; j < Y; j++)
    {
        for (int i = 0; i < X; i++)
        {
            maze[j][i].distance = calcCenter(i, j, X);
            maze[j][i].walls = 15;
            // If this is the left column (0,x)
            if (i == 0)
            {
                maze[j][i].walls = 7;
            }
            // if this is the top row
            if (j == 0)
            {
                maze[j][i].walls = 14;
            }
            // if this is the bottom row
            if (j == (Y - 1))
            {
                maze[j][i].walls = 13;
            }
            // If this is the righ column
            if (i == (X - 1))
            {
                maze[j][i].walls = 11;
            }
            maze[0][0].walls = 6;
            maze[Y - 1][0].walls = 5;
            maze[0][X - 1].walls = 10;
            maze[X - 1][Y - 1].walls = 9;
        }
    }
}

void MicroMouse::resetToCoord(coord desiredCoord)
{
    for (int j = 0; j < Y; j++)
    {
        for (int i = 0; i < X; i++)
        {
            maze[j][i].distance = calcDist(i, j, desiredCoord.x, desiredCoord.y);
        }
    }
}

// Get the most optimistic distance between two coordinates in a grid
int MicroMouse::calcDist(byte posx, byte posy, byte desireX, byte desireY)
{
    int dist = (int)(abs(desireY - posy) + abs(desireX - posx));
    return dist;
}

// Get the most optimistic distance between a given coordinate and a
// 2x2 square in the center of a maze of dimension dim (dim must be even)
int MicroMouse::calcCenter(byte posx, byte posy, byte dim)
{
    byte center = dim / 2;
    int dist = 0;

    if (posy < center)
    {
        if (posx < center)
        {
            // You're in the top left of the maze
            dist = calcDist(posx, posy, (center - 1), (center - 1));
        }
        else
        {
            // You're in the top right of the maze
            dist = calcDist(posx, posy, center, (center - 1));
        }
    }
    else
    {
        if (posx >= center)
        {
            // You're in the bottom right of the maze
            dist = calcDist(posx, posy, center, center);
        }
        else
        {
            // You're in the bottom left of the maze
            dist = calcDist(posx, posy, (center - 1), center);
        }
    }
    return dist;
}

/*
INPUT: a coordinate representing a current position, and a heading
OUTPUT: the coordinates of the next desired position based on the heading and current position
*/
MicroMouse::coord MicroMouse::bearingCoord(coord currCoord, byte heading)
{
    coord nextCoord = {0, 0};
    switch (heading)
    {
    case 1:
        // code
        nextCoord.x = currCoord.x;
        nextCoord.y = currCoord.y - 1;
        break;
    case 2:
        nextCoord.x = currCoord.x;
        nextCoord.y = currCoord.y + 1;
        break;
    case 4:
        nextCoord.x = currCoord.x + 1;
        nextCoord.y = currCoord.y;
        break;
    case 8:
        nextCoord.x = currCoord.x - 1;
        nextCoord.y = currCoord.y;
        break;
    }
    return nextCoord;
}

/*
INPUT: A Coord representing the current coordiante and the robots current heading
OUTPUT: An optimal direction away from the current coordinate.
*/
byte MicroMouse::orient(coord currCoord, byte heading)
{

    coord leastNext = {0, 0};
    // This is the absolute largest value possible (dimension of maze squared)
    int leastNextVal = sizeof(maze) * sizeof(maze);
    byte leastDir = heading;

    // If there is a bitwise equivalence between the current heading and the cell's value, then the next cell is accessible
    if ((maze[currCoord.x][currCoord.y].walls & heading) != 0)
    {
        // Define a coordinate for the next cell based onthis heading and set the leastNextVal t its value
        coord leastnextTemp = bearingCoord(currCoord, heading);

        if (checkBounds(leastnextTemp))
        {
            leastNext = leastnextTemp;
            leastNextVal = maze[leastNext.y][leastNext.x].distance;
        }
    }

    for (int i = 0; i < sizeof(headings); i++)
    {
        byte dir = headings[i];
        // if this dir is accessible
        if ((maze[currCoord.y][currCoord.x].walls & dir) != 0)
        {
            // define the coordiante for this dir
            coord dirCoord = bearingCoord(currCoord, dir);

            if (checkBounds(dirCoord))
            {
                // if this dir is more optimal than continuing straight
                if (maze[dirCoord.y][dirCoord.x].distance < leastNextVal)
                {
                    // update teh value of leastNextVal
                    leastNextVal = maze[dirCoord.y][dirCoord.x].distance;
                    // update the value of leastnext to this dir
                    leastNext = dirCoord;
                    leastDir = dir;
                }
            }
        }
    }
    return leastDir;
}

// Take a coordinate and test if it is within the allowable bounds
boolean MicroMouse::checkBounds(coord Coord)
{
    if ((Coord.x >= X) || (Coord.y >= Y) || (Coord.x < 0) || (Coord.y < 0))
    {
        return false;
    }
    else
    {
        return true;
    }
}

/*
INPUT: Coord
OUTPUT: An integer that is the least neighbor
*/
int MicroMouse::checkNeighs(coord Coord)
{
    int minVal = sizeof(maze) * sizeof(maze);
    for (int i = 0; i < sizeof(headings); i++)
    {
        byte dir = headings[i];
        // if this dir is accessible
        if ((maze[Coord.y][Coord.x].walls & dir) != 0)
        {
            // Get the coordinate of the accessible neighbor
            coord neighCoord = bearingCoord(Coord, dir);
            // Check the value of the accessible neighbor
            if (checkBounds(neighCoord))
            {
                // if the neighbore is less than the current recording minimum value, update the minimum value
                // If minVal is null, set it right away, otherwise test
                if (maze[neighCoord.y][neighCoord.x].distance < minVal)
                {
                    minVal = maze[neighCoord.y][neighCoord.x].distance;
                }
            }
        }
    }
    return minVal;
}

// Given a coordinate, test and return if the coordinate is bounded on three sides
boolean MicroMouse::isDead(coord coord)
{
    boolean deadEnd = false;
    if (checkBounds(coord))
    {
        byte bounds = maze[coord.y][coord.x].walls;
        // bounds is the integer from the exploratory maze that represents the known walls of the coordinate
        if ((bounds == 1) || (bounds == 2) || (bounds == 4) || (bounds == 8))
        {
            deadEnd = true;
        }
    }
    return deadEnd;
}

boolean MicroMouse::isEnd(coord Coord, coord DesiredArray[])
{
    boolean End = false;
    for (int i = 0; i < sizeof(DesiredArray); i++)
    {
        coord Desired = DesiredArray[i];
        if (checkBounds(Coord))
        {
            if ((Coord.x == Desired.x) && (Coord.y == Desired.y))
            {
                End = true;
            }
        }
    }
    return End;
}

/*
int readAhead(){
  return 0;
}
  */

/*
INPUT: Coordindate to update, and a direction representing the wall to add
OUTPUT: Update to coordinate adding the wall provided as an argument
*/

void MicroMouse::coordUpdate(coord coordinate, byte wallDir)
{
    if (checkBounds(coordinate))
    {
        if ((maze[coordinate.y][coordinate.x].walls & wallDir) != 0)
        {
            maze[coordinate.y][coordinate.x].walls = maze[coordinate.y][coordinate.x].walls - wallDir;
        }
    }
}

/*
INPUT: Current Robot coordinate
OUTPUT: Update maze for learned walls
*/
void MicroMouse::floodFillUpdate(coord currCoord, coord desired[])
{
    std::stack<coord> entries;

    maze[currCoord.y][currCoord.x].walls = readCurrent();
    entries.push(currCoord);

    for (int i = 0; i < sizeof(headings); i++)
    {
        byte dir = headings[i];
        // If there's a wall in this dir
        if ((maze[currCoord.y][currCoord.x].walls & dir) == 0)
        {
            // create a temporary working coordinate
            coord workingCoord = {currCoord.x, currCoord.y};
            switch (dir)
            {
            case 1:
                workingCoord.y = workingCoord.y - 1;
                coordUpdate(workingCoord, 2);
                break;
            case 2:
                workingCoord.y = workingCoord.y + 1;
                coordUpdate(workingCoord, 1);
                break;
            case 4:
                workingCoord.x = workingCoord.x + 1;
                coordUpdate(workingCoord, 8);
                break;
            case 8:
                workingCoord.x = workingCoord.x - 1;
                coordUpdate(workingCoord, 4);
                break;
            }
            // If the workingEntry is a valid entry and not a dead end, push it onto the stack
            if (checkBounds(workingCoord) && (!isEnd(workingCoord, desired)))
            {
                entries.push(workingCoord);
            }
        }
    }
    // While the entries stack isn't empty
    while (!entries.empty())
    {
        // Pop an entry from the stack
        coord workingEntry = entries.top();
        entries.pop();
        int neighCheck = checkNeighs(workingEntry);
        // If the least neighbor of the working entry is not one less than the value of the working entry
        if (neighCheck + 1 != maze[workingEntry.y][workingEntry.x].distance)
        {
            maze[workingEntry.y][workingEntry.x].distance = neighCheck + 1;
            for (int i = 0; i < sizeof(headings); i++)
            {
                byte dir = headings[i];
                if ((maze[workingEntry.y][workingEntry.x].walls & dir) != 0)
                {
                    coord nextCoord = bearingCoord(workingEntry, dir);
                    if (checkBounds(nextCoord))
                    {
                        if (!isEnd(nextCoord, desired))
                        {
                            entries.push(nextCoord);
                        }
                    }
                }
            }
        }
    }
}

// Break down and learn how this functions

void MicroMouse::floodFill(coord desired[], coord current, boolean isMoving)
{
    // coord desired[4] = {{X - 1, Y - 1}, {X - 1, Y}, {X, Y - 1}, {X, Y}};
    coord currCoord = current;
    byte heading = globalHeading;
    /*Integer representation of heading
     * 1 = N
     * 4 = E
     * 2 = S
     * 8 = W
     */
    while (maze[currCoord.y][currCoord.x].distance != 0)
    {
        floodFillUpdate(currCoord, desired);
        byte nextHeading = orient(currCoord, heading);
        coord nextCoord = bearingCoord(currCoord, nextHeading);

        /*     if (isMoving)
            {
              // Call createInstruction to push a new instruction to the stack
              instructions.push(createInstruction(currCoord, nextCoord, nextHeading));

              // Pop the next instruction from the instructions queue and execute it
              executeInstruction(instructions.pop());
            } */

        // After exectuing the instruction update the values of the local and global variables
        currCoord = nextCoord;
        heading = nextHeading;
        // If the robot has actually moved, update the global position variables
        if (isMoving)
        {
            globalHeading = heading;
            globalCoord = currCoord;
        }
    }
    // Set the global end as the current coordinate.
    globalEnd = currCoord;
}

/*
void loop()
{
  coord desired[] = {{X - 1, Y - 1}, {X - 1, Y}, {X, Y - 1}, {X, Y}};
  floodFill(desired, globalCoord, true);
  coord returnCoord[] = {{0, 0}};
  resetToCoord(returnCoord[0]);
  // Run fill to return to the start coord
  floodFill(returnCoord, globalCoord, true);

  // Reflood the maze
  reflood();

  // Pop instructions from the front of the queue until its empty.
  while (!instructions.isEmpty())
  {
    executeInstruction(instructions.pop());
  }
  */