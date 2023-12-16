#include "main.h"
#include "circularmotion.h"
#include "auto.h"
#include "pidClass.h"

double tileLengthMm = 24.0 * 25.4;
double wheelRevMm = 320.0;
double motorRevMm = 1 * (36.0 / 84.0) * wheelRevMm; // how many distance travelled per motor revolution
double rotateRadiusMm = 13.0 / 2 * 25.4;
// motorRev * motorRevMm = travelDistance
// motorRev = travelDistance / motorRevMm
double currentLRev = 0;
double currentRRev =0;

/**
 * @brief Drives the robot forward for a distance while turning towards a given angle.
 * 
 * @param distanceMm The total distance to travel.
 * @param targetangle The target angle to rotate to or keep the heading to.
 * @param velocityPct The linear velocity of the travel.
 * @param turnVelocityPct The angular velocity for the turning.
 * @param timeoutMs How long the function will drive the robot for.
*/
void driveForward(double distanceMm, double targetangle, double velocityPct, double turnVelocityPct, double timeoutMs = 3000) {
    timer timeout;
    timeout.reset();
    // Set PID
    PIDClass drivePid(37, 0, 0, 0.25);//10.0 is feasible
    PIDClass rotatePid(0.88, 0.000, 0, 3);
    // Calculate motor revolutions
    double motorRev = distanceMm / motorRevMm;
    velocityPct = fabs(velocityPct);
    double maxLinearPct = velocityPct;
    turnVelocityPct = fabs(turnVelocityPct);
    double maxTurnPct = turnVelocityPct;
    double initLeft = LeftMotors.position(rev);
    double initRight = RightMotors.position(rev);
    // task::sleep(100);
    // Spin until done or until timeout
    while (!drivePid.isSettled() && timeout.value() * 1000 < timeoutMs) {
        // Get velocity from PID
        double currentLRev = LeftMotors.position(rev) - initLeft;
        double currentRRev = RightMotors.position(rev) - initRight;
        double currentRev = (currentLRev + currentRRev) / 2;
        double error = motorRev - currentRev;
        drivePid.computeError(error);
        double newLinearVelocityPct = drivePid.getValue();

        double rotateError = targetangle - MJ.rotation();
        rotatePid.computeError(rotateError);
        double newAngularVelocityPct = rotatePid.getValue();
        
        // if (newVelocityPct > velocityPct) newVelocityPct = velocityPct;
        // else if (newVelocityPct < -velocityPct) newVelocityPct = -velocityPct;
        newLinearVelocityPct = fmin(maxLinearPct, fmax(-maxLinearPct, newLinearVelocityPct));
        newAngularVelocityPct = fmin(maxTurnPct, fmax(-maxTurnPct, newAngularVelocityPct));

        double leftVelocityPct = newLinearVelocityPct + newAngularVelocityPct;
        double rightVelocityPct = newLinearVelocityPct - newAngularVelocityPct;

        // Scale
        double scaleFactor = 100.0 / fmax(100.0, fmax(fabs(leftVelocityPct), fabs(rightVelocityPct)));
        leftVelocityPct *= scaleFactor;
        rightVelocityPct *= scaleFactor;

        printf("Error: %.3f, velocity: %.3f, terror: %.3f, tvel: %.3f\n", error, newLinearVelocityPct, rotateError, newAngularVelocityPct);
        
        LeftMotors.spin(fwd, leftVelocityPct, pct);
        RightMotors.spin(fwd, rightVelocityPct, pct);
        task::sleep(10);
    }
    // Stop
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

/// @param centerOffsetMm The offset of center of rotation. + for right, - for left.
void turnToAngle(double degree, double centerOffsetMm, double meanVelocityPct, double timeoutMs = 3000) {
    timer timeout;
    timeout.reset();
    // Compute radius (mm) for left and right
    double leftRadius = rotateRadiusMm + centerOffsetMm;
    double rightRadius = rotateRadiusMm - centerOffsetMm;
    double meanRadius = (leftRadius + rightRadius) / 2;
    // Calculate arc length (mm) to rotate
    double arcLengthLeft = (degree / 360) * (2 * M_PI * leftRadius);
    double arcLengthRight = (degree / 360) * (2 * M_PI * rightRadius);
    // Calculate motor revolutions to rotate
    double motorRevLeft = arcLengthLeft / motorRevMm;
    double motorRevRight = arcLengthRight / motorRevMm;
    // Set PID
    PIDClass rotatePid(0.85, 0.0005, 0, 3);
    // Variables for PID
    meanVelocityPct = fabs(meanVelocityPct);
    // double maxVolt = fmin(12.0, fmax(-12.0, meanVelocityPct / 100.0 * 12.0));
    double initLeft = LeftMotors.position(rev);
    double initRight = RightMotors.position(rev);
    // Spin until done or until timeout
    while (!rotatePid.isSettled() && timeout.value() * 1000 < timeoutMs) {
        // Calculate error
        /*currentLRev = LeftMotors.position(rev) - initLeft;
        currentRRev = -(RightMotors.position(rev) - initRight);
        double currentLMm = currentLRev * motorRevMm;
        double currentRMm = currentRRev * motorRevMm;
        double angRad;
        if (fabs(leftRadius) <= 1e-3) {
            // Use right radius
            // radian * radius = arc length
            // radian = length / radius
            angRad = currentRMm / rightRadius;
        } else if (fabs(rightRadius) <= 1e-3) {
            angRad = currentLMm / leftRadius;
        } else {
            double rightRad = (currentRMm / rightRadius);
            double leftRad = (currentLMm / leftRadius);
            angRad = (rightRad + leftRad) / 2;
        }
        double angDeg = angRad * 180 / M_PI;*/
        double error = degree - MJ.rotation();
        // Get velocity from PID
        rotatePid.computeError(error);
        double newVelocityPct = fmin(meanVelocityPct, fmax(-meanVelocityPct, rotatePid.getValue()));
        // Calculate velocities
        double leftVelocityPct = newVelocityPct * (leftRadius / meanRadius);
        double rightVelocityPct = newVelocityPct * (rightRadius / meanRadius);
        // Scale
        double scaleFactor = 100.0 / fmax(100.0, fmax(fabs(leftVelocityPct), fabs(rightVelocityPct)));
        leftVelocityPct *= scaleFactor;
        rightVelocityPct *= scaleFactor;
        printf("Current deg: %.3f, error: %.3f, lefV: %.3f, rgtV: %.3f\n", MJ.rotation(), error, leftVelocityPct, rightVelocityPct);
        // Spin
        LeftMotors.spin(forward, leftVelocityPct, pct);
        RightMotors.spin(reverse, rightVelocityPct, pct);

        task::sleep(10);
    }
    // Stop
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
}

void resetAngle(double rotation){
    MJ.setRotation(rotation, deg);
}


void intakeSetState(int state) {
    if (state == 1) {
        IntakeMotor.spin(reverse, 100, pct);//out
    } else if (state == -1) {
        IntakeMotor.spin(fwd, 100, pct);//in
    } else {
        IntakeMotor.stop(coast);
    }
}

void autonomousggSkill() {
    // Intake middle triball
    //tim skills ver1.0
    /*resetAngle(115);
    CatapultMotors.spinToPosition(370,deg,-100,rpm,true);//lift catapult to let the intake out
    //push two alli-triballs
    driveForward(1.3 * tileLengthMm, 180, 100, 28, 1100);
    driveForward(-0.76 * tileLengthMm, 82, 100, 70);
    driveForward(0.4*tileLengthMm, 82, 100, 100);//into matchload position
    throwMotor.spin(reverse, 11, volt);//matchload
    //task::sleep(30000);//30 secs
    CatapultMotors.spinTo(30,deg,-100, rpm,true);//lift down, end
    throwMotor.stop(coast);
    //part I - start pushing balls from the side (under elevation bar)
    turnToAngle(132, 0, 90);
    driveForward(-2.15*tileLengthMm, 90, 100, 15);
    driveForward(-2.9*tileLengthMm, 0, 100, 9);
    turnToAngle(0, 0, 100);
    driveForward(-0.5*tileLengthMm, 0, 100, 100);
    driveForward(0.5*tileLengthMm, 0, 100, 100);
    driveForward(-0.5*tileLengthMm, 0, 100, 100);
    driveForward(-2*tileLengthMm, 0, 100, 100);
    //part II - middle triballs
    dig1.set(1);
    driveForward(-1.6*tileLengthMm, -63, 50, 100);
    driveForward(-1*tileLengthMm, 0, 60, 10);
    turnToAngle(90, 0, 100);
    driveForward(-1*tileLengthMm, 90, 100, 30);
    driveForward(0.5*tileLengthMm, 90, 100, 100);
    driveForward(-0.5*tileLengthMm, 90, 100, 100);
    driveForward(0.8*tileLengthMm, 180, 60, 10);
    */
    //tim  skills vers2.0
    resetAngle(-48);
    turnUp();//lift catapult to let the intake out
    //push two alli-triballs
    driveForward(-1.5 * tileLengthMm, 0, 100, 28, 1100);
    driveForward(0.9 * tileLengthMm, -95, 100, 70);
    driveForward(-0.5*tileLengthMm, -95, 100, 100);//into matchload position
    turnDown();//lift down, end
    throwMotor.spin(reverse, 12, volt);//matchload
    //task::sleep(30000);//30 secs
    throwMotor.stop(coast);
    //part I - start pushing balls from the side (under elevation bar)
    turnToAngle(-48, 0, 100);
    driveForward(2.21*tileLengthMm, -90, 100, 15);
    driveForward(2.53*tileLengthMm, -180, 90, 5);
    driveForward(1.0*tileLengthMm, -180, 100, 100, 1000);
    driveForward(-0.32*tileLengthMm, -180, 100, 100);
    driveForward(1.0*tileLengthMm, -180, 100, 100, 1000);
    driveForward(-0.39*tileLengthMm, -142, 100, 100);
    //part II - middle triballs
    dig1.set(1);
    driveForward(-1.6*tileLengthMm, -72, 30, 90);
    driveForward(-1.9*tileLengthMm, 90, 40, 30);//first push with wings
    dig1.set(0);
    driveForward(1.3*tileLengthMm, 120, 45, 45);
    dig1.set(1);
    driveForward(-1.4*tileLengthMm, 80, 90, 30);//second push with wings
    driveForward(1.5*tileLengthMm, 110, 45, 45);
    driveForward(-2*tileLengthMm, 30, 45, 40);
    //part III
    dig1.set(0);
    driveForward(-1.5*tileLengthMm, 180, 100, 60);
    driveForward(0.32*tileLengthMm, 180, 100, 100);
    driveForward(-0.4*tileLengthMm, 170, 100, 90);
    /*dig1.set(0);
    //part I - start pushing balls from the side (under elevation bar)
    turnToAngle(-48, 0, 100);
    driveForward(-2.5*tileLengthMm, -90, 100, 15);
    driveForward(-1.8*tileLengthMm, -140, 80, 5);
    driveForward(-1.3 * tileLengthMm, -180, 60, 60);
    driveForward(0.32*tileLengthMm, -180, 100, 100);
    driveForward(-0.38*tileLengthMm, -180, 100, 100);
    driveForward(0.3*tileLengthMm, -180, 100, 100);
    turnToAngle(-48, 0, 100);
    driveForward(1.8 * tileLengthMm, -35, 100, 100);
    turnToAngle(-90, 0, 100);
    //part II - middle triballs
    dig1.set(1);
    driveForward(-1.9*tileLengthMm, -90, 70, 40);//first push with wings
    dig1.set(0);
    driveForward(1.7 * tileLengthMm, -80, 45, 5);
    turnToAngle(-90, -rotateRadiusMm, 100);
    dig1.set(1);
    driveForward(-1.9*tileLengthMm, -90, 90, 30);//second push with wings
    dig1.set(0);
    driveForward(1.5*tileLengthMm, -90, 45, 45);
    dig1.set(1);
    driveForward(-1.7*tileLengthMm, -90, 45, 40);
    //part III
    dig1.set(0);
    driveForward(-1.5*tileLengthMm, 180, 100, 60);
    driveForward(0.32*tileLengthMm, 180, 100, 100);
    driveForward(-0.4*tileLengthMm, 170, 100, 90);*/

/*Ree
    resetAngle(-61);
    driveForward(-1.3 * tileLengthMm, 0, 100, 28, 1100);
   // driveForward(0.8 * tileLengthMm, -20, 100, 100);
    driveForward(1.1 * tileLengthMm, -40, 100, 70);
    driveForward(0.4*tileLengthMm, 82, 100, 100);
    throwMotor.spin(reverse, 11, volt);
    CatapultMotors.spinToPosition(370,deg,-100,rpm,true);
    task::sleep(32000);
    CatapultMotors.spinToPosition(-70,deg,-100,rpm,true);
    throwMotor.stop(coast);
    driveForward(-1 * tileLengthMm, 120, 100, 100);
    driveForward(-2.5 * tileLengthMm, 90, 100, 100);
    driveForward(-1 * tileLengthMm, 0, 100, 100);
    driveForward( 0.5 * tileLengthMm, 100, 70, 60);
    driveForward(0.7 * tileLengthMm, 90, 100, 100);
    dig1.set(1);
    driveForward(-1 * tileLengthMm, 90, 100, 100);
    dig1.set(0);
    driveForward( 1.5 * tileLengthMm, 280,  100, 60);
    driveForward(0.7 * tileLengthMm, 270, 100, 100);
    driveForward(-0.7 * tileLengthMm, 270, 100, 100);

*/

}


void autonTest() {
    resetAngle(0);
    driveForward(1 * tileLengthMm, 90.0, 100.0, 100.0);
    // driveForward(1 * tileLengthMm, 0, 100.0, 100.0);
    // driveForward(1 * tileLengthMm, 0, 100.0, 100.0);
    // driveForward(-1 * tileLengthMm, 0, 100.0, 100.0);
    // driveForward(-1 * tileLengthMm, 0, 100.0, 100.0);
    // driveForward(2 * tileLengthMm, 0, 100.0, 100.0);
    // driveForward(-2 * tileLengthMm, 0, 100.0, 100.0);
}