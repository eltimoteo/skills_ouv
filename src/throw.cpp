#include "main.h"
#include "throw.h"

int x=1;
int y=1;
int spd =85;
void keybindThrow() {
  Controller1.ButtonA.pressed(throwstatechange);
  Controller1.ButtonB.pressed(throwstatechange2);

                                                                                                                                                                                                    
}

void throwstatechange() { 
    x += 1;

    if(x%2==0){
        // throwMotor.spin(forward, spd, pct);
        double spinVolt = fmin(12.0, fmax(-12.0, spd / 100.0 * 12.0));
        throwMotor.spin(forward, spinVolt, volt);
    }
    else if(x%2 == 1) {
        throwMotor.stop(coast);
    }
}

void throwstatechange2() { 
    y += 1;
    
    if(y%2==0){
        // throwMotor.spin(reverse, spd, pct);
        double spinVolt = fmin(12.0, fmax(-12.0, spd / 100.0 * 12.0));
        throwMotor.spin(reverse, spinVolt, volt);
    }
    else if(y%2 == 1) {
        throwMotor.stop(coast);
    }
}

