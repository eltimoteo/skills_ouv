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
    PIDClass drivePid(13.5, 0, 0, 0.2);//10.0 is feasible
    PIDClass rotatePid(0.30, 0.0005, 0, 3);
    // Calculate motor revolutions
    double motorRev = distanceMm / motorRevMm;
    velocityPct = fabs(velocityPct);
    double maxVolt = fmin(12.0, fmax(-12.0, velocityPct / 100.0 * 12.0));
    turnVelocityPct = fabs(turnVelocityPct);
    double maxTurnVolt = fmin(12.0, fmax(-12.0, turnVelocityPct / 100.0 * 12.0));
    double initLeft = LeftMotors.position(rev);
    double initRight = RightMotors.position(rev);
    // task::sleep(100);
    // Spin until done or until timeout
    while (!drivePid.isSettled() && timeout.value() * 1000 < timeoutMs) {
        double currentLRev = LeftMotors.position(rev) - initLeft;
        double currentRRev = RightMotors.position(rev) - initRight;
        double currentRev = (currentLRev + currentRRev) / 2;
        double error = motorRev - currentRev;
        drivePid.computeError(error);
        double newLinearVelocityVolt = drivePid.getValue();

        double rotateError = targetangle - MJ.rotation();
        // Get velocity from PID
        rotatePid.computeError(rotateError);
        double newAngularVelocityVolt = rotatePid.getValue();
        
        // if (newVelocityPct > velocityPct) newVelocityPct = velocityPct;
        // else if (newVelocityPct < -velocityPct) newVelocityPct = -velocityPct;
        newLinearVelocityVolt = fmin(maxVolt, fmax(-maxVolt, newLinearVelocityVolt));
        newAngularVelocityVolt = fmin(maxTurnVolt, fmax(-maxTurnVolt, newAngularVelocityVolt));

        double leftVelocityVolt = newLinearVelocityVolt + newAngularVelocityVolt;
        double rightVelocityVolt = newLinearVelocityVolt - newAngularVelocityVolt;

        // Scale
        double scaleFactor = 12.0 / fmax(12.0, fmax(fabs(leftVelocityVolt), fabs(rightVelocityVolt)));
        leftVelocityVolt *= scaleFactor;
        rightVelocityVolt *= scaleFactor;

        printf("Error: %.3f, velocity: %.3f\n", error, newLinearVelocityVolt);
        
        LeftMotors.spin(fwd, leftVelocityVolt, volt);
        RightMotors.spin(fwd, rightVelocityVolt, volt);
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
    
    resetAngle(-45);
    // Put down intake
    throwMotor.spin(reverse, 100, pct);  
    // Intake center-middle triball
    IntakeMotor.spin(fwd, 100, pct);
    driveForward(1.3 * tileLengthMm, -37, 100, 100);
    throwMotor.stop(coast);
    turnToAngle(-30, 0,100);
    driveForward(1.4 * tileLengthMm, -30, 100, 100);
    //driveForward(1.3*tileLengthMm, -30, 100, 100);
    // Score loaded and center-right balls
    turnToAngle(90, 0, 100);
    intakeSetState(1);
    driveForward(1.2 * tileLengthMm, 90, 100, 100, 1100);
    // Intake center-low ball
    driveForward(-0.4 * tileLengthMm, 90, 100, 100);
    turnToAngle(233, 0, 100);
    intakeSetState(-1);
    driveForward(0.8 * tileLengthMm, 233, 100, 100);
    // Go the match load zone
    turnToAngle(123, 0, 100);
    driveForward(2.05 * tileLengthMm, 145, 100, 100);
    // Release loaded triball toward goal
    turnToAngle(30, 0, 100);
    intakeSetState(1);
    task::sleep(300);
    intakeSetState(0);
    //push three triballs
    turnToAngle(-110, 0, 100);
    dig1.set(1);
    driveForward(-0.4 * tileLengthMm, -130, 100, 60);
    dig1.set(0);
    driveForward(-1.5 * tileLengthMm, -180, 100, 16);
    //driveForward(-0.3 * tileLengthMm, -180, 100, 100);
    // turnToAngle(270, rotateRadiusMm * 2, 100);
    // intakeSetState(-1);
    // driveForward(0.83 * tileLengthMm, 270, 100, 100);
    // turnToAngle(90, 0, 100);
    // driveForward(1.2 * tileLengthMm, 90, 100, 100);
    // turnToAngle(45, rotateRadiusMm*-1, 100);
}

