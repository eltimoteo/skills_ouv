#include "main.h"
#include "botDrive.h"

void ggDriver() {
    // Get axis positions
    double axis3 = Controller1.Axis3.position(pct);
    double axis1 = Controller1.Axis1.position(pct);
    if (fabs(axis3) <= 5) axis3 = 0;
    if (fabs(axis1) <= 5) axis1 = 0;
    // Get left and right percentages
    double leftPct = axis3 + axis1;
    double rightPct = axis3 - axis1;
    double scale = 100.0 / fmax(100.0, fmax(fabs(leftPct), fabs(rightPct)));
    leftPct *= scale;
    rightPct *= scale;
    
    if(fabs(fabs(leftPct)- fabs(rightPct)>40)){
        LeftMotors.setStopping(hold);
        RightMotors.setStopping(hold);
    }
    else{
        LeftMotors.setStopping(brake);
        RightMotors.setStopping(brake);
    }
    
    
    // printf("L: %.3f, R: %.3f\n", LeftMotors.velocity(pct), RightMotors.velocity(pct));
    
    RightMotors.spin(forward, (rightPct), pct);
    LeftMotors.spin(forward, (leftPct), pct);
}