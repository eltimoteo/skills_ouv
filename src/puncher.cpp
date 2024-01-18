#include "main.h"
#include "puncher.h"

int x = 1;

int spd = 85;
void keybindPuncher()
{
    Controller1.ButtonA.pressed(puncherStateChange);
   
}


void puncherStateChange()
{
    x += 1;

    if (x % 2 == 0)
    {
        // throwMotor.spin(forward, spd, pct);
        double motorVolt = fmin(12.0, fmax(-12.0, spd / 100.0 * 12.0));
        puncherMotor.spin(forward, motorVolt, volt);
    }
    else if (x % 2 == 1)
    {
        puncherMotor.stop(coast);
    }
}

