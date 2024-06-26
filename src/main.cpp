//----------------------------------------------------------------------------//
/*                                                                            */
//    Module:       main.cpp                                                  //
//    Author:       C:\Users\user                                             //
//    Created:      Tue Jul 11 2023                                           //
//    Description:  V5 project                                                //
//                                                                            //
//----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightMotors          motor_group   11, 12
// LeftMotors           motor_group   13, 14
// IntakeMotor              motor         17
// CatapultMotors       motor_group   15, 16
// Controller1          controller
//numa                  digital_out   A
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "main.h"
#include "botDrive.h"
#include "pneumatics.h"
#include "puncher.h"

#include "auto.h"

competition Competition;

double boom = 0;
int count = 0;


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  //vexcodeInit();
  task::sleep(1100);

  task puncherTask([] () -> int {
    puncherThread();
    return 1;
  });

  MJ.startCalibration();
  while (MJ.isCalibrating())
  {
    task::sleep(100);
  }
}

bool alreadyResetCata = false;
void autonomous(void) {
  // resetCatapult();
  // alreadyResetCata = true;
  LeftMotors.setStopping(brake);
  RightMotors.setStopping(brake);
  //autonomousggFar();
  // autonomousggSkill();
  autonomousggSkill();
}

void usercontrol(void) {
  PuncherMotors.setStopping(coast);
  LeftMotors.setStopping(brake);
  RightMotors.setStopping(brake);
  Brain.resetTimer();
  resetCatapult();

 
  keybindPneumatics();
  keybindPuncher();

  while (true) {
    ggDriver();
    // if(Controller1.ButtonR2.pressing()){
    //   IntakeMotor.spin(forward, 100, pct);
    // }
    // else if(Controller1.ButtonR1.pressing()){
    //   IntakeMotor.spin(reverse, 100, pct);
    // }
    // else {
    //   IntakeMotor.stop(hold);
    // }
    
  }
}

int main() {

  pre_auton();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  while(1){
    task::sleep(100);
  } 
}
