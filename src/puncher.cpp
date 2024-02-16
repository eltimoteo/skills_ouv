#include "main.h"
#include "puncher.h"

namespace {
    double cataRevDeg = 360.0;
    double cataSlipToothDeg = 360.0 / 36.0;
    void puncherStateChange();

    void resetPuncherFunction();
    void puncherOneRevolution();
    void spinPuncherToAngle(double degrees);

    bool x = 1;
    int spd = 85;

    int puncherRevolutionCount = 0;
    int punchedCount = 0;

    bool puncherSpinDebounce = false;
    bool isPuncherResetted = false;
    bool canPuncherRun = true;
}

void keybindPuncher()
{
    Controller1.ButtonA.pressed(puncherOneRevolution);
    puncherRevolutionCount = 0;
}

void resetCatapult()
{
    if (isPuncherResetted) {
        return;
    }
    task resetPuncherTask([] () -> int {
        resetPuncherFunction();
        return 1;
    });
}

void puncherThread() {
    // Detect balls
    punchedCount = 0;
    while (true) {
        if(isPuncherResetted && canPuncherRun && DS.objectDistance(mm) < 20) {
            puncherOneRevolution();
            punchedCount++;
        }
        task::sleep(10);
    }
}

int getPunchedCount() {
    return punchedCount;
}

void setPuncherCanRun(bool canRun) {
    canPuncherRun = canRun;
}

namespace {
    void resetPuncherFunction() {
        isPuncherResetted = false;

        // Spin to slip part
        PuncherMotors.resetPosition();

        timer timeout;
        timeout.reset();

        // Nonslip to slip
        PuncherMotors.spin(fwd, 40, pct);
        timeout.reset();
        task::sleep(100);
        while (PuncherMotors.torque() > 0.10 && timeout.value() <= 2)
        {
            // printf("Torque: %.3f\n", PuncherMotors.torque());
            task::sleep(10);
        }
        printf("At slip\n");
        /* Slip gear is at the start of the slip section */

        // Spin to the start of nonslip
        PuncherMotors.setMaxTorque(0.10, Nm);
        task::sleep(300);
        PuncherMotors.stop();
        printf("At nonslip\n");

        // Spin catapult to bottom
        PuncherMotors.setMaxTorque(100, pct);
        PuncherMotors.resetPosition();
        spinPuncherToAngle(360.0 * (6.0 / 36.0));
        PuncherMotors.stop(hold);
        PuncherMotors.resetPosition();

        isPuncherResetted = true;
    }
    void puncherOneRevolution()
    {
        if (!puncherSpinDebounce)
        {
            puncherSpinDebounce = true;

            puncherRevolutionCount++;
            spinPuncherToAngle(puncherRevolutionCount * 120);
            // spinPuncherToAngle(puncherRevolutionCount * (360*5/3));
        
            puncherSpinDebounce = false;
        }
    }
    void spinPuncherToAngle(double degrees)
    {
        PuncherMotors.spin(fwd, 11, volt);
        timer runTimeout;
        while (PuncherMotors.position(deg) < degrees && runTimeout.value() < 0.5)
        {
            task::sleep(10);
        }
        printf("Target: %.3f, motor: %.3f\n", degrees, PuncherMotors.position(deg));
        PuncherMotors.stop(hold);
    }
}