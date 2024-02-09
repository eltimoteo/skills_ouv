#include "main.h"
#include "puncher.h"

double cataRevDeg = 360.0;
double cataSlipToothDeg = 360.0 / 36.0;
void puncherStateChange();

    void spinPuncherToAngle(double degrees);

    bool x = 1;
    int spd = 85;

    int puncherRevolutionCount = 0;

    bool puncherSpinDebounce = false;

void keybindPuncher()
{
    Controller1.ButtonA.pressed(puncherOneRevolution);
    puncherRevolutionCount = 0;
}

void puncherOneRevolution()
{
    if (!puncherSpinDebounce)
    {
        puncherSpinDebounce = true;

        puncherRevolutionCount++;
        spinPuncherToAngle(puncherRevolutionCount * (360*5/3));
     

        puncherSpinDebounce = false;
    }
}
void resetCatapult()
{
  // Spin to slip part
  PuncherMotors.resetPosition();

  timer timeout;
  timeout.reset();
  // Slip to nonslip
  PuncherMotors.spin(fwd, 50, pct);
  while (PuncherMotors.torque() <= 0.10 && timeout.value() <= 2)
  {
    task::sleep(10);
  }
  // Nonslip to slip
  timeout.reset();
  while (PuncherMotors.torque() > 0.10 && timeout.value() <= 2)
  {
    task::sleep(10);
  }
  /* Slip gear is at the start of the slip section */

  // Spin to the start of nonslip
  PuncherMotors.setMaxTorque(0.03, Nm);
  PuncherMotors.stop();
  // Spin catapult to bottom
  PuncherMotors.setMaxTorque(100, pct);

  task::sleep(700);
  PuncherMotors.stop(hold);
}

    void spinPuncherToAngle(double degrees)
    {
        PuncherMotors.resetPosition();
        PuncherMotors.spin(fwd, 12, volt);
        timer runTimeout;
        while (PuncherMotors.position(deg) < degrees && runTimeout.value() < 0.5)
        {
            task::sleep(10);
        }
        PuncherMotors.stop(hold);
    }