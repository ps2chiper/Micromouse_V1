#ifndef MICROMOUSE_H_
#define MICROMOUSE_H_

#include <Arduino.h>
#include "RobotCarPinDefinitionsAndMore.h"
#include "CarPWMMotorControl.h"
#include "SparkFun_SHTC3.h"
#include "HCSR04.h"
#include <stack>

class MicroMouse
{
public:
    static inline SHTC3 mySHTC3;

    static const short MAX_DISTANCE = 200;
    static inline UltraSonicDistanceSensor sensorArray[3] = {
        UltraSonicDistanceSensor(PIN_TRIGGER_OUT_LEFT, PIN_ECHO_IN_LEFT, MAX_DISTANCE),
        UltraSonicDistanceSensor(PIN_TRIGGER_OUT_FRONT, PIN_ECHO_IN_FRONT, MAX_DISTANCE),
        UltraSonicDistanceSensor(PIN_TRIGGER_OUT_RIGHT, PIN_ECHO_IN_RIGHT, MAX_DISTANCE)};

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
    const int baseSpeed = 120;
    const float P = 0.7;
    const float D = 0.5;
    const float I = 0.4;

    // What is the offset for?
    const int offset = 5;

    const int wall_threshold = 13;
    // int left_threshold = 10 ;
    // int right_threshold = 10 ;
    const int front_threshold = 7;
    const unsigned int pingSpeed = 30; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.

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

private:
    int dir;
    boolean wall[3];
    boolean first_turn;
    boolean rightWallFollow;
    boolean leftWallFollow;
    float oldErrorP;
    float totalError;
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
        Front = 1,
        Right = 2,
        New = 0,
        Average = 1,
        Old = 2
    };
    int RMS;
    int LMS;
    unsigned long pingTimer; // Holds the next ping time.

    float Sensor[3][3];
    float temperatureCentigrade;

    // Move around
    //--------------------------------- control ---------------------------------//
    //--------------------------------- direction control ---------------------------------//

    void setDirection(int dir);
    //---------------------------------------------------------------------------//

    //--------------------------------- Sensors ---------------------------------//

    void ReadSensors();
    //---------------------------------------------------------------------------//

    //----------------------------- wall follow  control -------------------------------//

    void PID(boolean left);
    //--------------------------- wall detection --------------------------------//

    void walls();

    //---------------------------------------------------------------------------//

    void turnright();

    //---------------------------------------------------------------------------//

    void turnleft();

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
    coord desired[4];// = {{X - 1, Y - 1}, {X - 1, Y}, {X, Y - 1}, {X, Y}};

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