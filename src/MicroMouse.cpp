#include "MicroMouse.h"

MicroMouse Runner;

MicroMouse::MicroMouse(float temp)
    : first_turn(false), rightWallFollow(false), leftWallFollow(false), temperatureCentigrade(temp)
{
    Wire.setClock(WIRE_SPEED);
    Wire.begin(SDA, SCL);
    delay(100);
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
    myPID.SetMode(myPID.Control::automatic);
    myPID.SetOutputLimits(-10, 10);
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);
    // Serial.println(F("Shutdown pins inited..."));
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    Serial.println(F("Both in reset mode...(pins are low)"));
    Serial.println(F("Starting..."));
    setID();
    lox1.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
    lox2.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
    lox1.startRangeContinuous();
    lox2.startRangeContinuous();
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
        RobotCarPWMMotorControl.leftCarMotor.setSpeedPWM(LMS, DIRECTION_FORWARD);
        RobotCarPWMMotorControl.rightCarMotor.setSpeedPWM(RMS, DIRECTION_FORWARD);
        Serial.println("FORWARD");
    }
    else if (dir == LEFT)
    {
        RobotCarPWMMotorControl.leftCarMotor.setSpeedPWM(LMS, DIRECTION_BACKWARD);
        RobotCarPWMMotorControl.rightCarMotor.setSpeedPWM(RMS, DIRECTION_FORWARD);
        Serial.println("LEFT");
    }
    else if (dir == RIGHT)
    {
        RobotCarPWMMotorControl.leftCarMotor.setSpeedPWM(LMS, DIRECTION_FORWARD);
        RobotCarPWMMotorControl.rightCarMotor.setSpeedPWM(RMS, DIRECTION_BACKWARD);
        Serial.println("RIGHT");
    }
    else if (dir == STOP)
    {
        RobotCarPWMMotorControl.leftCarMotor.stop(MOTOR_BRAKE);
        RobotCarPWMMotorControl.rightCarMotor.stop(MOTOR_BRAKE);
        Serial.println("STOP");
    }
    else if (dir == BACKWARD)
    {
        RobotCarPWMMotorControl.leftCarMotor.setSpeedPWM(LMS, DIRECTION_BACKWARD);
        RobotCarPWMMotorControl.rightCarMotor.setSpeedPWM(RMS, DIRECTION_BACKWARD);
        Serial.println("BACKWARD");
    }
}

void MicroMouse::ReadSensors()
{

    Sensor[Front][Old] = Sensor[Front][Average] = (SonicSensor.ping_cm(MAX_DISTANCE) + Sensor[Front][Old]) / 2.0;
    Sensor[Left][Average] = lox1.readRange();
    Sensor[Right][Average] = lox2.readRange();
    Sensor[Left][Average] = Sensor[Left][Average] > 20 ? Sensor[Left][Average] - 20 : 0;
    Sensor[Right][Average] = Sensor[Right][Average] > 20 ? Sensor[Right][Average] - 20 : 0;
}

void MicroMouse::PIDcalculate(boolean left)
{
    // Add variable for wall error.
    Setpoint = (wall[Left] && wall[Right] ? 0 : 41.5); // Setpoint of 0 if both walls are true or 41.5
    while (Sensor[Left][Average] < emergency_threshold && !wall[Right] && !wall[Front])
    {
        Serial.println(F("Emergency Turn!"));
        setDirection(STOP);
        // delay(500);
        ReadSensors();
        float degrees = atan((Sensor[Left][Average] - (Setpoint + (Setpoint - Sensor[Left][Average]))) / (float)offset) * RAD_TO_DEG;
        Serial.print(F("Turn Degrees: "));
        Serial.print(degrees);
        RobotCarPWMMotorControl.rotate(degrees, TURN_IN_PLACE);
        // delay(500);
        ReadSensors();
    }

    while (Sensor[Right][Average] < emergency_threshold && !wall[Left] && !wall[Front])
    {
        Serial.println(F("Emergency Turn!"));
        setDirection(STOP);
        // delay(500);
        ReadSensors();
        float degrees = atan((Sensor[Right][Average] + (Setpoint + (Setpoint - Sensor[Right][Average]))) / (float)offset) * RAD_TO_DEG;
        Serial.print(F("Turn Degrees: "));
        Serial.print(degrees);
        RobotCarPWMMotorControl.rotate(degrees, TURN_IN_PLACE);
        // delay(500);
        ReadSensors();
    }

    while ((Sensor[Left][Average] < emergency_threshold || Sensor[Right][Average] < emergency_threshold) && wall[Left] && wall[Right] && !wall[Front])
    {
        Serial.println(F("Emergency Turn!"));
        setDirection(STOP);
        // delay(500);
        ReadSensors();
        float degrees = atan((Sensor[Left][Average] - Sensor[Right][Average]) / (float)offset) * RAD_TO_DEG;
        Serial.print(F("Turn Degrees: "));
        Serial.print(degrees);
        RobotCarPWMMotorControl.rotate(degrees, TURN_IN_PLACE);
        // delay(500);
        ReadSensors();
    }

    // Need to add a go straight using without any walls.

    Setpoint = (wall[Left] && wall[Right] ? 0 : 41.5);
    Input = (wall[Left] ? Sensor[Left][Average] : 0) - (wall[Right] ? Sensor[Right][Average] : 0);

    double gap = abs(Setpoint - Input); // distance away from setpoint
    Serial.print("Gap: ");
    Serial.println(gap);
    if (gap < 15)
    { // we're close to setpoint, use conservative tuning parameters
        // myPID.SetOutputLimits(-5, 5);
        myPID.SetTunings(consKp, consKi, consKd);
    }
    /*     else if (gap < 10)
        { // we're close to setpoint, use conservative tuning parameters
            myPID.SetOutputLimits(-15, 15);
            myPID.SetTunings(consKp, consKi, consKd);
        }
        else if (gap < 20)
        { // we're close to setpoint, use conservative tuning parameters
            myPID.SetOutputLimits(-30, 30);
            myPID.SetTunings(consKp, consKi,consKd);
        }
        else if (gap < 30)
        { // we're close to setpoint, use conservative tuning parameters
            myPID.SetOutputLimits(-50, 50);
            myPID.SetTunings(aggKp, aggKi, aggKd);
        } */
    else
    {
        // we're far from setpoint, use aggressive tuning parameters
        // myPID.SetOutputLimits(-15, 15);
        myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    myPID.Compute();
    RMS = baseSpeed[Right] - Output;
    LMS = baseSpeed[Left] + Output;
    Serial.print("Output: ");
    Serial.println(Output);

    if (false)
    {
        if (Sensor[Left][Average] <= 40)
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
    if (!wall[Front])
    {

        setDirection(FORWARD);
        // delay(100);
    }
    else
    {
        setDirection(STOP);
    }
    // convert to switch
    // feed info into the floodfill algorithim to find heading.
    // break down into smooth turns.
    // left T section
    /*     if (wall[left] && !wall[Front] && !wall[Right])
        {
        }
        // dead end
        else if (wall[left] && wall[Front] && wall[Right])
        {
        }
        // four way section
        else if (!wall[left] && !wall[Front] && !wall[Right])
        {
        }
        // right T section
        else if (!wall[left] && !wall[Front] && wall[Right])
        {
        }
        // right only
        else if (wall[left] && wall[Front] && !wall[Right])
        {
        }
        // T section
        else if (!wall[left] && wall[Front] && !wall[Right])
        {
        }
        // target found
        // else if (!wall[left] && wall[Front] && !wall[Right])
        {
        }
        // left only
        else if (!wall[left] && wall[Front] && wall[Right])
        {
        } */
}

void MicroMouse::walls()
{

    // Just assign the boolean result of the test directly to the variable.
    wall[Left] = Sensor[Left][Average] < wall_threshold;
    wall[Right] = Sensor[Right][Average] < wall_threshold;
    wall[Front] = Sensor[Front][Average] < front_threshold && Sensor[Front][Average] >= 2.0;
    // walls_truth_table= wall[Left] * 100 + wall[Front] * 10 + wall[Right];
    // Serial.print(F("Wall truth table: "));
    // Serial.println(walls_truth_table);
}

void MicroMouse::turnright()
{

    /*     LMS = baseSpeed; */

    // RMS = LMS * Sensor[Right][Average] / (Sensor[Right][Average] + 11);

    RobotCarPWMMotorControl.goDistanceMillimeter(100);
    RobotCarPWMMotorControl.rotate(-90, TURN_FORWARD);
    RobotCarPWMMotorControl.goDistanceMillimeter(50);
}

void MicroMouse::turnleft()
{
    /*
        RMS = baseSpeed; */

    // LMS = RMS * Sensor[Left][Average] / (Sensor[Left][Average] + 11);
    do
    {
        ReadSensors();
        walls();
        PIDcalculate(false);
    } while (wall[Left]);
    RobotCarPWMMotorControl.goDistanceMillimeter(100);
    RobotCarPWMMotorControl.rotate(90, TURN_FORWARD);
    RobotCarPWMMotorControl.goDistanceMillimeter(50);
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
    Serial.print(" mm ");
    Serial.print(" Right : ");
    Serial.print(Sensor[Right][Average]);
    Serial.print(" mm ");
    Serial.print(" Front : ");
    Serial.print(Sensor[Front][Average]);
    Serial.println(" cm ");

    // measure error & print the result to the serial monitor

    // Serial.print("error=");
    // Serial.println(totalError);
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

void MicroMouse::setID()
{
    // all reset
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    delay(10);
    // all unreset
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    // activating LOX1 and resetting LOX2
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);

    // initing LOX1
    if (!lox1.begin(LOX1_ADDRESS))
    {
        Serial.println(F("Failed to boot first VL53L0X"));
        while (1)
            ;
    }
    delay(10);

    // activating LOX2
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    // initing LOX2
    if (!lox2.begin(LOX2_ADDRESS))
    {
        Serial.println(F("Failed to boot second VL53L0X"));
        while (1)
            ;
    }
}

/* void MicroMouse::read_dual_sensors()
{

    lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
    lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

    // print sensor one reading
    Serial.print(F("1: "));
    if (measure1.RangeStatus != 4)
    { // if not out of range
        Serial.print(measure1.RangeMilliMeter);
    }
    else
    {
        Serial.print(F("Out of range"));
    }

    Serial.print(F(" "));

    // print sensor two reading
    Serial.print(F("2: "));
    if (measure2.RangeStatus != 4)
    {
        Serial.print(measure2.RangeMilliMeter);
    }
    else
    {
        Serial.print(F("Out of range"));
    }

    Serial.println();
} */