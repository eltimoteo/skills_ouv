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
#include "throw.h"

#include "auto.h"

competition Competition;

double boom = 0;
int count = 0;
bool gay = false;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  //vexcodeInit();
  MJ.startCalibration();
  while(MJ.isCalibrating()){
    task::sleep(10);
  }

  dig2.set(0);
  CatapultMotors.stop(hold);
}

bool alreadyResetCata = false;
void autonomous(void) {
  gay = true;
  // resetCatapult();
  // alreadyResetCata = true;
  timer benchmark;
  CatapultMotors.setStopping(hold);
  LeftMotors.setStopping(brake);
  RightMotors.setStopping(brake);
  //autonomousggSkill();
  autonomousggSkill();
  // autonTest();
  printf("Time: %.3f s\n", benchmark.value());
}

void usercontrol(void) {
  CatapultMotors.spinToPosition(10,deg,-100,rpm,false);
  throwMotor.setStopping(coast);
  LeftMotors.setStopping(brake);
  RightMotors.setStopping(brake);
  CatapultMotors.setStopping(hold);
  Brain.resetTimer();
  

  keybindCatapult();
  keybindPneumatics();
  keybindThrow();

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
    if(gay==true){
      if( Brain.timer(timeUnits::sec) > 60 && Brain.timer(timeUnits::sec)<75) {
        Controller1.rumble("--. .- -.--");//print "GAY" with morse code
    
      }
      else if( Brain.timer(timeUnits::sec) >80 ){
        Controller1.rumble(".- ... - .-. --- / .. ... / --. .- -.--");//Print "Astro is Gay" with morse code 
    }
  }
    
   task::sleep(30); 
} 
    
  }



int main() {
  //group 13 is dy left
  //group 11 is dy right

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while(1){
    task::sleep(100);
  } 
}
