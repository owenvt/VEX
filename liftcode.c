#pragma config(Sensor, in4,    rightPot,         sensorAnalog)
#pragma config(Sensor, in5,    leftPot,         sensorAnalog)
#pragma config(Motor,  port5,           liftR,         tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port6,           liftL,         tmotorVex393, openLoop)
//motor and sensor definitions

/**********************************************************************************
*   Lift control sofware for VEX robots
*   Copyright (C) 2014  Owen Marshall
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*   Connact me at owenvt@gmail.com or as Owen on the Vex forums with
*   questions and comments.
**********************************************************************************/

//competition control
#pragma platform(VEX)
#pragma competitionControl(Competition)
#pragma autonomousDuration(45)
#pragma userControlDuration(75)
#include "Vex_Competition_Includes.c"

//this is the delay built into infinate loops
#define LOOP_DELAY  10

//global lift height definitions for your particular robot
#define GROUND    550
#define LOW       1300
#define MED       1800
#define HIGH      2500

//global variable array for ramping desired motor values
int rampSpeed[10] = {0,0,0,0,0,0,0,0,0,0};

//acceleration task for easy ramping of all motors
task rampMotors()
{
  //constants to avoid magic numbers
  const unsigned int NUMBER_OF_MOTORS = 10;
  const int MAX_SPEED = 127;
  const int MIN_SPEED = -127;
  const unsigned int RAMP_SPEED = 1;

  while(true)
  {
    //loop through all the moters
    for(int port=0; port<NUMBER_OF_MOTORS; port++)
    {
      //if the motor is too low, raise it
      if(rampSpeed[port] > motor[port])
        motor[port] = motor[port] + 1;

      //if the motor is to too high, lower it
      else if(rampSpeed[port] < motor[port])
        motor[port] = motor[port] - 1;

      //if the motor is more than the max speed, fix it
      if(motor[port] > MAX_SPEED)
        motor[port] = MAX_SPEED;

      //if the motor is less than the min speed, fix it
      else if(motor[port] < MIN_SPEED)
        motor[port] = MIN_SPEED;
    }
    //delay to smooth motor ramping
    wait1Msec(RAMP_SPEED);
  }
}

//sets right lift power
void rightLiftPower(int power)
{
  rampSpeed[liftR] = power;
}

//sets left lift power
void leftLiftPower(int power)
{
  rampSpeed[liftL] = power;
}

//sets the power for both sides of the lift
void liftPower(int power)
{
  rightLiftPower(power);
  leftLiftPower(power);
}

//ensures the lift is not set too high or two low
int boundLift(int target)
{
  if(target > HIGH)
    return HIGH;
  else if(target < GROUND)
    return GROUND;
  else
    return target;
}

//holds the lift at a specified height
void liftHold(int targetHeight)
{
  //initalize kp for going up and down, can be tuned as needed
  //offset is the baseline differnce between the two potentiometer readings
  const float KPU = 0.6, KPD = 0.2;
  const int offset = 30;

  //get the potentiometer values
  int leftAngle = SensorValue(leftPot) - offset;
  int rightAngle = SensorValue(rightPot);

  //ensure the input falls in the correct range
  targetHeight = boundLift(targetHeight);

  //determine the error
  int leftError = targetHeight - leftAngle;
  int rightError = targetHeight - rightAngle;

  //use a p control to set motors
  if(rightError > 0)
    rampSpeed[liftR] = rightError * KPU;  //if error is positive, use the up kp
  else
    rampSpeed[liftR] = rightError * KPD;  //if error is negitive, use the down kp

  if(leftError > 0)
    rampSpeed[liftL] = leftError * KPU; //if error is positive, use the up kp
  else
    rampSpeed[liftL] = leftError * KPD; //if error is negitive, use the down kp
}

//global lift setting for control in autonoumous
int autoLiftSet = GROUND;
//task lift runs during autonomous and allows easy control of the lift
task autoLift()
{
  while(true)
  {
    liftHold(autoLiftSet);
  }
  //delay to allow background processes to run
  wait1Msec(LOOP_DELAY);
}

//code to run at startup
void pre_auton()
{
  bStopTasksBetweenModes = false;
  StartTask(rampMotors);  //this task should run at all times
}

//autonomous code
task autonomous()
{
  StartTask(autoLift);  //this allows control of the lift in autonoumous while driving simultaniously

  //examples of lift control during autonomous
  autoLiftSet = MED;
  autoLiftSet = 1337;
}

//user control code
//note that drive or intake code can be added with no modification
//(it is possible to drive and lift at the same time without multitasking)
task usercontrol()
{
  //stop tasks
  StopTask(autonomous);
  StopTask(autoLift);   //disables autonomous lift control and enables manual control

  //lift control contstants
  const int MANUAL = 0, SAFETY = 50, MANUAL_SPEED_UP = 127, MANUAL_SPEED_DOWN = -127, velocityThreshold = 1;

  //lift control variables
  int setting=MANUAL, targetHeight=MANUAL, currentHeight, oldHeight, speed;
  bool stopFlag = false;

  while(true)
  {
    //get the current height and calculate speed
    oldHeight = currentHeight;
    currentHeight = SensorValue(rightPot);
    speed = currentHeight - oldHeight;

    //up button pressed
    if(vexRT(Btn6U))
    {
      //manual raise
      if (currentHeight > HIGH-SAFETY)
      {
        //if the lift is already up, raise it no farther
        liftHold(HIGH);
      }
      else
      {
        //if it isn't, continue as normal
        liftPower(MANUAL_SPEED_UP);
      }
      //temporarily disable button set heights
      setting = MANUAL;
      stopFlag = true;
    }

    //down button pressed
    else if(vexRT(Btn6D))
    {
      //manual lower
      if (currentHeight < GROUND+SAFETY)
      {
        //if the lift is already down, stop lowering it
        liftHold(GROUND);
      }
      else
      {
        //otherwise continue as normal
        liftPower(MANUAL_SPEED_DOWN);
      }
      //temporarily disable button set heights
      setting = MANUAL;
      stopFlag = true;
    }

    //button released and button liftholds not active
    else if(setting == MANUAL)
    {
      //brake lift
      if(stopFlag)
      {
        //wait for the lift to stop
        liftPower(0);
        if(abs(speed) < velocityThreshold)
        {
          //the lift has come to a stop more or less
          stopFlag = false;
          targetHeight = currentHeight;
        }
      }

      //the lift has finished stopping
      else
      {
        //this will always activate unless no buttons have been pressed
        if(targetHeight)
        {
          //hold the lift at the desired height
          liftHold(targetHeight);
        }
        else
        {
          //targetHeight can be set to 0 or MANUAL at any time to disable the liftHold
          liftPower(0);
        }
      }
    }

    //button liftholds which temporarily disable the manual lifthold
    //manual can be re-engaged at any time
    if(vexRT(Btn7L))
      setting = GROUND;

    else if(vexRT(Btn7D))
      setting = LOW;

    else if(vexRT(Btn7R))
      setting = MED;

    else if(vexRT(Btn7U))
      setting = HIGH;

    //if the setting is not manual, set the lift height
    if(setting)
      liftHold(setting);

    //delay to allow background processes to run
    wait1Msec(LOOP_DELAY);
  }
}
