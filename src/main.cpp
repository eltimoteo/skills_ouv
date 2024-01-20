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
#include "circularmotion.h"
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
  MJ.startCalibration();
  while (MJ.isCalibrating())
  {
    task::sleep(100);
  }


  CatapultMotors.stop(hold);
}

bool alreadyResetCata = false;
void autonomous(void) {
  // resetCatapult();
  // alreadyResetCata = true;
  CatapultMotors.setStopping(hold);
  LeftMotors.setStopping(brake);
  RightMotors.setStopping(brake);
  //autonomousggFar();
  // autonomousggSkill();
  autonomousggSkill();
}

void usercontrol(void) {
  CatapultMotors.spinToPosition(10,deg,-100,rpm,false);
  puncherMotor.setStopping(coast);
  LeftMotors.setStopping(brake);
  RightMotors.setStopping(brake);
  CatapultMotors.setStopping(hold);
 Brain.resetTimer();
  

 
  keybindPneumatics();
  keybindPuncher();

  while (true) {
    ggDriver();
    if(Controller1.ButtonR2.pressing()){
      IntakeMotor.spin(forward, 100, pct);
    }
    else if(Controller1.ButtonR1.pressing()){
      IntakeMotor.spin(reverse, 100, pct);
    }
    else {
      IntakeMotor.stop(hold);
    }
    
  }
}

int main() {
  //group 15 is the angrybird.io
  //group 13 is dy left
  //group 11 is dy right

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while(1){
    task::sleep(100);
  } 
}
