#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
motor RightMotorsMotorA = motor(PORT19, ratio6_1, false);
motor RightMotorsMotorB = motor(PORT12, ratio6_1, false);
motor RightMotorsMotorC = motor(PORT11, ratio6_1, false);
motor_group RightMotors = motor_group(RightMotorsMotorA, RightMotorsMotorB, RightMotorsMotorC);
motor LeftMotorsMotorA = motor(PORT8, ratio6_1, true);
motor LeftMotorsMotorB = motor(PORT14, ratio6_1, true);
motor LeftMotorsMotorC = motor(PORT18, ratio6_1, true);
motor_group LeftMotors = motor_group(LeftMotorsMotorA, LeftMotorsMotorB, LeftMotorsMotorC);
motor IntakeMotor = motor(PORT17, ratio6_1, false); 
motor CatapultMotorsMotorA = motor(PORT15, ratio36_1, false);
motor CatapultMotorsMotorB = motor(PORT16, ratio36_1, true);
motor_group CatapultMotors = motor_group(CatapultMotorsMotorA, CatapultMotorsMotorB);
controller Controller1 = controller(primary);
digital_out wings = digital_out(Brain.ThreeWirePort.H);
digital_out hang = digital_out(Brain.ThreeWirePort.A);
digital_out blocker = digital_out(Brain.ThreeWirePort.B);
motor puncherMotor = motor(PORT10, ratio36_1, true);
inertial MJ = inertial(PORT1);


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool Controller1UpDownButtonsControlMotorsStopped = true;
bool Controller1XBButtonsControlMotorsStopped = true;

// define a task that will handle monitoring inputs from Controller1
