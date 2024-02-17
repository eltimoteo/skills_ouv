#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

motor RightMotorsMotorA = motor(PORT20, ratio6_1, false);
motor RightMotorsMotorB = motor(PORT19, ratio6_1, false);
motor RightMotorsMotorC = motor(PORT18, ratio6_1, false);
motor_group RightMotors = motor_group(RightMotorsMotorA, RightMotorsMotorB, RightMotorsMotorC);

motor LeftMotorsMotorA = motor(PORT17, ratio6_1, true);
motor LeftMotorsMotorB = motor(PORT16, ratio6_1, true);
motor LeftMotorsMotorC = motor(PORT15, ratio6_1, true);
motor_group LeftMotors = motor_group(LeftMotorsMotorA, LeftMotorsMotorB, LeftMotorsMotorC);

motor IntakeMotor = motor(PORT4, ratio6_1, false); 

digital_out wings = digital_out(Brain.ThreeWirePort.E);
digital_out backWingR = digital_out(Brain.ThreeWirePort.G);
digital_out hang = digital_out(Brain.ThreeWirePort.H);
digital_out backWingL = digital_out(Brain.ThreeWirePort.F);

motor PuncherMotorA = motor(PORT13, ratio36_1, true);
motor PuncherMotorB = motor(PORT21, ratio36_1);
motor_group PuncherMotors = motor_group(PuncherMotorA, PuncherMotorB);

inertial MJ = inertial(PORT12);

distance DS = distance(PORT14);


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool Controller1UpDownButtonsControlMotorsStopped = true;
bool Controller1XBButtonsControlMotorsStopped = true;

// define a task that will handle monitoring inputs from Controller1
