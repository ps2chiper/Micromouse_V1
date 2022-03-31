#ifndef MICROMOUSE_H_
#define MICROMOUSE_H_

#include <Arduino.h>
#include "RobotCarPinDefinitionsAndMore.h"
#include "CarPWMMotorControl.h"
#include "SparkFun_SHTC3.h"
#include <stack>
#include <NewPing.h>
#include "QuickPID.h"
#include "Adafruit_VL53L0X.h"

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 PB3
#define SHT_LOX2 PA15

class MicroMouse
{
public:
    // Make sure to addjust some of these to prive.
    float Setpoint, Input, Output;
    // double Kp = 1.4, Ki = 0, Kd = 0;
    const double aggKp = 4, aggKi = 0.2, aggKd = 1;
    const double consKp = 1, consKi = 0.05, consKd = 0.25;
    // float Kp = 2, Ki = 5, Kd = 1;

    QuickPID myPID = QuickPID(&Input, &Output, &Setpoint);

    // objects for the vl53l0x
    static inline Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
    static inline Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

    // this holds the measurement
    static inline VL53L0X_RangingMeasurementData_t measure1;
    static inline VL53L0X_RangingMeasurementData_t measure2;

    static inline SHTC3 mySHTC3;

    static const short MAX_DISTANCE = 50;
    static inline NewPing SonicSensor = NewPing(PIN_TRIGGER_OUT_FRONT, PIN_ECHO_IN_FRONT, MAX_DISTANCE);

    // Adjust to be your own pins.
    /*
    #define TRIGGER_PINL  A3  // Arduino pin tied to trigger pin on ping sensor.
    #define ECHO_PINL     A0  // Arduino pin tied to echo pin on ping sensor.

    #define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

    #define TRIGGER_PINF  A4  // Arduino pin tied to trigger pin on ping sensor.
    #define ECHO_PINF     A1  // Arduino pin tied to echo pin on ping sensor.

    #define TRIGGER_PINR  A5  // Arduino pin tied to trigger pin on ping sensor.
    #define ECHO_PINR     A2  // Arduino pin tied to echo pin on ping sensor.
    */

    // Switching to a const enum list
    /*
    #define STOP 0
    #define FORWARD 1
    #define BACKWARD 2
    #define LEFT 3
    #define RIGHT 4
    */
    // Set base speed to make the micromouse move.
    unsigned long SensorPing[3];

    const int baseSpeed[2] = {100, 100};
    // const float P = 0.7;
    // const float D = 0.5;
    // const float I = 0.4;

    // Offest is the distance between the sensors.
    const int offset = 152;

    // Ememergency turn threshold
    const int emergency_threshold = 30;

    const int wall_threshold = 200; // MM for Time of Flight Sensors.
    // int left_threshold = 10 ;
    // int right_threshold = 10 ;
    const int front_threshold = 5; // CM for UltraSonic Sensors.

    // boolean frontwall;
    // boolean leftwall;
    // boolean rightwall;

    // int en1 = 2;
    // int en2 = 3;

    // int en3 = 4;
    // int en4 = 5;

    // int enA = 10;
    // int enB = 11;

    // int baseSpeed = 70;

    // int LED = 13 ;
    // int led1 = 8 ;
    // int led2 = 9 ;

    // UltraSonicDistanceSensor *arrayOptr[3];
    //  Move this someplace else later on.

    // int TestNUM = 1  ;

    MicroMouse(float temp = 19.307 /* Default value for distance sensors */);
    virtual ~MicroMouse();

    void init();

    float getTempC() const;

    void runInLoop();

    void setTempC(float temp);
    void read_dual_sensors();

    void turnright();

    //---------------------------------------------------------------------------//

    void turnleft();

    //---------------------------------------------------------------------------//

private:
    void setID();
    int dir;
    byte walls_truth_table;
    boolean wall[4];
    boolean first_turn;
    boolean rightWallFollow;
    boolean leftWallFollow;

    enum direction
    {
        STOP,
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    };

    enum sonarSensor
    {
        Left = 0,
        Right = 1,
        Front = 2,
        New = 0,
        Average = 1,
        Old = 2
    };
    int RMS;
    int LMS;
    unsigned long pingTimer; // Holds the next ping time.

    float Sensor[3][3];
    float temperatureCentigrade, humidity, soundsp, soundcm;

    // Move around
    //--------------------------------- control ---------------------------------//
    //--------------------------------- direction control ---------------------------------//

    void setDirection(int dir);
    //---------------------------------------------------------------------------//

    //--------------------------------- Sensors ---------------------------------//

    void ReadSensors();
    //---------------------------------------------------------------------------//

    //----------------------------- wall follow  control -------------------------------//

    void PIDcalculate(boolean left);
    //--------------------------- wall detection --------------------------------//

    void walls();

    //---------------------------------------------------------------------------//

    // void pid_start();

    void updateTempC();

    void errorDecoder(SHTC3_Status_TypeDef message);

    // Begin floodfill algorithim

    struct instruction
    {
        float desiredPos;
        float desiredHeading;
    };

    struct coord
    {
        int x;
        int y;
    };

    struct entry
    {
        int distance;
        int walls;
    };

    byte readCurrent();
    byte globalHeading = 4;
    instruction createInstruction(coord currCoord, coord nextCoord, byte nextHeading);
    coord globalCoord = {0, 0};
    coord globalEnd = {0, 0};

    // Test this
    coord desired[4]; // = {{X - 1, Y - 1}, {X - 1, Y}, {X, Y - 1}, {X, Y}};

    // Define some global constants
    static constexpr uint8_t X = 16;
    static constexpr uint8_t Y = 16;

    entry maze[Y][X];
    // N,S,E,W
    const byte headings[4] = {1, 2, 4, 8};
    void instantiateReflood();
    void instantiate();
    void resetToCoord(coord desiredCoord);
    int calcDist(byte posx, byte posy, byte desireX, byte desireY);
    int calcCenter(byte posx, byte posy, byte dim);
    coord bearingCoord(coord currCoord, byte heading);
    byte orient(coord currCoord, byte heading);
    boolean checkBounds(coord Coord);
    int checkNeighs(coord Coord);
    boolean isDead(coord coord);
    boolean isEnd(coord Coord, coord DesiredArray[]);
    void coordUpdate(coord coordinate, byte wallDir);
    void floodFillUpdate(coord currCoord, coord desired[]);
    void floodFill(coord desired[], coord current, boolean isMoving);
};

extern MicroMouse Runner;

#endif /* MICROMOUSE_H_ */
#pragma once