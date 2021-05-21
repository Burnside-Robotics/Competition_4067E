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



float pusharmspeed = 100;
float motorspeed = 100;

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

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


bool enablePID = true;

int drivePID() {

  while(enablePID){

    //Postion of both motors 
    int leftMotorPostion = motor_left.position(degrees);
    int rightMotorPostion = motor_right.position(degrees);


    //Lateral Movement Pid


    

    //average position
    int averagePostion= (leftMotorPostion + rightMotorPostion) / 2;
    
    //potential
    error = averagePostion - desiredValue;

    //deriative
    derivative = error - prevError;

    //integral 
    //totalError += error;

    int MotorPower = error * kP + derivative * kD;

    motor_left.spin(reverse, MotorPower, pct);
    motor_left.spin(fwd, motorspeed, pct);

    //turning movement PID
    //average position
    int turnDifference = (leftMotorPostion - rightMotorPostion);
    
    //potential
    turnError = turnDifference - desiredTurnValue ;

    //deriative
    turnDerivative = turnError - turnPrevError;

    //integral 
    //turnTotalError += turnError;

    int turnMotorPower = turnError * turnkP + turnDerivative * turnkD;
    

    motor_left.spin(reverse, turnMotorPower, pct);
    motor_right.spin(forward, turnMotorPower, pct);


    
    //code
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }


  return 1; 
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

  vex::task yeet(drivePID);
  desiredValue = 50;
  desiredTurnValue = 80;


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
  enablePID = false; 

    
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
        
    
      if (Controller1.Axis3.position() - Controller1.Axis2.position() > 50)  {
        motorspeed = 0.85;
        //if bot joystick position turning, limit speed
      }
      else if (Controller1.Axis2.position() - Controller1.Axis3.position() > 50) {
        motorspeed = 0.85;
        //if bot joystick position turning, limit spee (opposite)
      }
      else {
        motorspeed = 1;
      }
        motor_left.spin(vex::directionType::fwd, Controller1.Axis3.position(vex::percentUnits::pct) * motorspeed, vex::velocityUnits::pct);
        motor_right.spin(vex::directionType::rev, Controller1.Axis2.position(vex::percentUnits::pct) * motorspeed, vex::velocityUnits::pct);   

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}






//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
