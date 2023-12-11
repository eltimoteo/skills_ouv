#include "pneumatics.h"
#include "main.h"

namespace {
    void switchWingsState();
    // void switchWingsState2();
    void switchRachetState();

    int y = 1;
    bool wingsDebounce = false;
}

void keybindPneumatics() {
    Controller1.ButtonY.pressed(switchWingsState);
    Controller1.ButtonUp.pressed(switchRachetState);
}

namespace {
    void switchWingsState() {
        if (!wingsDebounce) {
            wingsDebounce = true;
            dig1.set(!dig1.value());
            
            task::sleep(100);
            wingsDebounce = false;
        }
    }

    // void switchWingsState2() {
    //     dig2.set(0);
    // }
    void switchRachetState() {
        dig2.set(1);
    }
}