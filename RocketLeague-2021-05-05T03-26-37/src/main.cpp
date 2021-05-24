/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

motor motor_left (PORT1, ratio18_1);
motor motor_right (PORT10, ratio18_1);

controller Controller1;

float lSpeed = 0;
float rSpeed = 0;

float pusharmspeed = 100;
//float motorspeed = 100;

//settings
double kP = 0.25;
double kI = 0.0;
double kD = 0.1;

double turnkP = 0.2;
double turnkI = 0.0;
double turnkD = 0.1;

//autonomous settings
int desiredValue = 20;
int desiredTurnValue = 0;

int error;//sensorvalue - desiredvalue : position
int prevError = 0; //postion 20 msec ago
int derivative;//error - preverror : speed
int totalError;

int turnError;//sensorvalue - desiredvalue : position
int turnPrevError = 0; //postion 20 msec ago
int turnDerivative;//error - preverror : speed
int turnTotalError;

const float WHEEL_CIRCUMFERENCE = 31.9185812596;
float MOTOR_ACCEL_LIMIT = 100;

int s_lastL = 0;
int s_lastR = 0; 

void ToggleDriveDirection()
{
  lSpeed = -10;
}

void setSideSpeeds(int lSpeed, int rSpeed)
{
    if ((lSpeed - s_lastL) > MOTOR_ACCEL_LIMIT)
        lSpeed = s_lastL + MOTOR_ACCEL_LIMIT;
    if ((lSpeed - s_lastL) < -MOTOR_ACCEL_LIMIT)
        lSpeed = s_lastL - MOTOR_ACCEL_LIMIT;
    if ((rSpeed - s_lastR) > MOTOR_ACCEL_LIMIT)
        rSpeed = s_lastR + MOTOR_ACCEL_LIMIT;
    if ((rSpeed - s_lastR) < -MOTOR_ACCEL_LIMIT)
        rSpeed = s_lastR - MOTOR_ACCEL_LIMIT;

    s_lastL = lSpeed;
    s_lastR = rSpeed;

    if (lSpeed == 0)
        motor_left.stop(brakeType::brake);
    else
        motor_left.spin(directionType::fwd, lSpeed, velocityUnits::pct);
    if (rSpeed == 0)
        motor_right.stop(brakeType::brake);
    else
        motor_right.spin(directionType::fwd, rSpeed, velocityUnits::pct);
}
// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void DriveDistance(int dist, float maxTime)
{
  motor_left.resetPosition();
  motor_right.resetPosition();

  //Constant Tuning Values
  const float Kp = 1;
  const float Kd = 0;
  const float Ki = 0;

  float rotationGoal = (dist / WHEEL_CIRCUMFERENCE) * 360;

  const float maxSpeed = 100;
  const float accelTime = 500;


  float distError = 0;
  float integral = 0;
  float derivative = 0;
  float lastError = 0;

  float motorSpeed = 0;
  
  float doneTime = 0;
  while(maxTime > doneTime / 1000)
  {
    distError = rotationGoal - motor_left.rotation(deg);

    integral += distError;

    if(distError > 200 || distError < -200)
    {
      integral  = 0;
    }

    derivative = distError - lastError;

    lastError = distError;

    motorSpeed = Kp * distError + Ki * integral + Kd * derivative;
  }
}




void UpdateScreen() {
  Controller1.Screen.clearScreen();

  if (Competition.isDriverControl()) 
  {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("User Control Active");
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Battery: ");
    Controller1.Screen.print(Brain.Battery.capacity());
    Controller1.Screen.print("%%");
  }
  else if (Competition.isAutonomous()) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Autonomous");
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Battery: ");
    Controller1.Screen.print(Brain.Battery.capacity());
    Controller1.Screen.print("%%");
  }

}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {


  
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  DriveDistance(50, 5);



}



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  

    
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
        



      if (Controller1.Axis3.position() - Controller1.Axis2.position() > 50)  {
        MOTOR_ACCEL_LIMIT = 80;
        
        //if bot joystick position turning, limit speed
      }
      else if (Controller1.Axis2.position() - Controller1.Axis3.position() > 50) {
        MOTOR_ACCEL_LIMIT = 80;
        //if bot joystick position turning, limit spee (opposite)
      }
      else {
        MOTOR_ACCEL_LIMIT = 100;
        
      }
           
    

      
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }

}





//
// Main will set up the competition functions and callbacks.
//
int main() 
{
  Controller1.ButtonA.pressed(ToggleDriveDirection);
 // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  UpdateScreen();

  // Run the pre-autonomous function.
  

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
