#include "circularmotion.h"
#include "main.h"

double cataRevDeg = 360.0;
double cataSlipToothDeg = 360.0 / 36.0;

// void onCatapultRotate();
// void onCatapultRelease();

void resetCatapult()
{
}

void keybindCatapult() {
  Controller1.ButtonL1.pressed(turnUp);
  Controller1.ButtonL2.pressed(turnDown);

}

bool catapultDebounce = false;
void turnUp()
{
  //Serial.print("Test print at turn up");

  printf("Test turn Up");
  timer timeout;
  // CatapultMotors.spinTo(150,deg,false);
  // task::sleep(1000);
  // CatapultMotors.stop(hold);
  CatapultMotors.spinToPosition(350,deg,-100,rpm,true);
  
  CatapultMotors.stop(hold);

 
}

void turnDown() {
 timer timeout;
 printf("Test turn down");
  
  // CatapultMotors.spinToPosition(0,deg,-100,rpm,false);
  // task::sleep(1000);
  // CatapultMotors.stop(hold);
  CatapultMotors.spinToPosition(-40,deg,-100, rpm,true);
  
  CatapultMotors.stop(hold);
  
  

}
