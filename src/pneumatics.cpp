#include "pneumatics.h"
#include "main.h"

namespace
{
    void switchWingsState();
    void switchBackWingsState();
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
    Controller1.ButtonX.pressed(switchBackWingsState);
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
            hang1.set(!hang1.value());
            hang2.set(!hang2.value());

            task::sleep(100);
            hangDebounce = false;
        }
    }

    void switchBackWingsState()
    {
        if (!blockerDebounce)
        {
            blockerDebounce = true;
            backWings.set(!backWings.value());

            task::sleep(100);
            blockerDebounce = false;
        }
    }
}