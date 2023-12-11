using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor_group RightMotors;
extern motor_group LeftMotors;
extern motor IntakeMotor;
extern motor_group CatapultMotors;
extern controller Controller1;
extern digital_out dig1;
extern digital_out dig2;
extern motor throwMotor;
extern inertial MJ;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void );