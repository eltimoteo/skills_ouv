#include "pneumatics.h"
#include "main.h"

namespace
{
    void switchWingsState();
    void switchBlockerState();
    void switchHangState();

    int y = 1;
    bool wingsDebounce = false;
    bool blockerDebounce = false;
    bool hangDebounce = false;
}

void keybindPneumatics()
{
    Controller1.ButtonY.pressed(switchWingsState);
    Controller1.ButtonUp.pressed(switchHangState);
    Controller1.ButtonX.pressed(switchBlockerState);
}

namespace
{
    void switchWingsState()
    {
        if (!wingsDebounce)
        {
            wingsDebounce = true;
            wings.set(!wings.value());

            task::sleep(100);
            wingsDebounce = false;
        }
    }

    // void switchWingsState2() {
    //     dig2.set(0);
    // }
    void switchHangState()
    {
       if (!hangDebounce)
        {
            hangDebounce = true;
            hang.set(!hang.value());

            task::sleep(100);
            hangDebounce = false;
        }
    }

    void switchBlockerState()
    {
        if (!blockerDebounce)
        {
            blockerDebounce = true;
            blocker.set(!blocker.value());

            task::sleep(100);
            blockerDebounce = false;
        }
    }
}