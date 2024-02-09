using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor_group RightMotors;
extern motor_group LeftMotors;
extern motor IntakeMotor;
extern controller Controller1;
extern digital_out wings;
extern digital_out hang;
extern digital_out backWings;
extern motor_group PuncherMotors;
extern inertial MJ;
extern distance DS;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void );