#include <Arduino.h>
#include "RobotCarPinDefinitionsAndMore.h"
#include "CarPWMMotorControl.hpp"

#include "MicroMouse.h"

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */

#define SPEED_PWM_COMPENSATION_RIGHT 0
#define SIZE_OF_SQUARE_MILLIMETER 400

// MicroMouse Runner;

void errorDecoder(SHTC3_Status_TypeDef message);

void setup()
{
  
  // initialize the digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  long sTimer = millis() + 4000L;
  while (sTimer > millis())
  {
    if (Serial)
    {
      break;
    }
  }

  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

  RobotCarPWMMotorControl.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_INTERRUPT, LEFT_MOTOR_FORWARD_PIN,
                               LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_INTERRUPT);

  RobotCarPWMMotorControl.setDriveSpeedPWM(120);

  // Print info
  PWMDcMotor::printSettings(&Serial);

  Runner.init();
  Serial.print("TempC is: ");
  Serial.println(Runner.getTempC());

  Runner.turnleft();
  //delay(1000);
  //Runner.turnright(); 
}

void loop()
{

  
  //Runner.runInLoop();
}

