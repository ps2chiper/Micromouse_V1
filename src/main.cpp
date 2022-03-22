/*
 *  Square.cpp
 *  Example for driving a 50 cm square using CarPWMMotorControl class
 *
 *  Copyright (C) 2020-2021  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/PWMMotorControl.
 *
 *  PWMMotorControl is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <FreeRTOSConfig_Default.h>
#include "RobotCarPinDefinitionsAndMore.h"
#include "CarPWMMotorControl.hpp"
#include "SparkFun_SHTC3.h"
#include "HCSR04.h"

#include "MicroMouse.h"

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */

#define SPEED_PWM_COMPENSATION_RIGHT 0
#define SIZE_OF_SQUARE_MILLIMETER 400

// MicroMouse Runner;

void Thread1(void *pvParameters);
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
  // RobotCarPWMMotorControl.leftCarMotor.init(LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_INTERRUPT);
  // RobotCarPWMMotorControl.rightCarMotor.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_INTERRUPT);

  // RobotCarPWMMotorControl.setDriveSpeedAndSpeedCompensationPWM(DEFAULT_DRIVE_SPEED_PWM, SPEED_PWM_COMPENSATION_RIGHT); // Set compensation
  // RobotCarPWMMotorControl.setFactorDegreeToMillimeter(FACTOR_DEGREE_TO_MILLIMETER_2WD_CAR_DEFAULT);

  // Print info
  PWMDcMotor::printSettings(&Serial);
  // RobotCarPWMMotorControl.rightCarMotor.wheelGoDistanceTicks(100L, 255, DIRECTION_FORWARD);
  // RobotCarPWMMotorControl.leftCarMotor.wheelGoDistanceTicks(100L, 255, DIRECTION_FORWARD);
  // delay(1000);
  //RobotCarPWMMotorControl.leftCarMotor.setSpeedPWM(255, DIRECTION_FORWARD);
  // RobotCarPWMMotorControl.rightCarMotor.setSpeedPWM(255, DIRECTION_FORWARD);
  // RobotCarPWMMotorControl.leftCarMotor.updateDriveSpeedPWM(255);
  // RobotCarPWMMotorControl.leftCarMotor.start(DIRECTION_FORWARD);
  // RobotCarPWMMotorControl.leftCarMotor.wheelGoDistanceTicks(100L, 255, DIRECTION_FORWARD);
  // delay(1000);

  // RobotCarPWMMotorControl.rotate(-90, TURN_IN_PLACE);

  Runner.init();
  Serial.print("TempC is: ");
  Serial.println(Runner.getTempC());

  xTaskCreate(
      Thread1, (const portCHAR *)"Main Thread" // A name just for humans
      ,
      128 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL);

  // start scheduler
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while (1)
    ;
}

void loop()
{
  // Not used
}

void Thread1(void *pvParameters __attribute__((unused))) // This is a Task.
{
  while (true)
  {
    /*     Runner.ReadSensors();
        Serial.print("Left Sensor: ");
        Serial.println(Runner.leftSensor);
        Serial.print("Front Sensor: ");
        Serial.println(Runner.frontSensor);
        Serial.print("Right Sensor: ");
        Serial.println(Runner.rightSensor); */
    // vTaskDelay(1); // use instead of delay
    Runner.runInLoop();
  }
}
