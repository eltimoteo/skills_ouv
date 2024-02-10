#include "main.h"
#include "puncher.h"
#include "auto.h"
#include "pidClass.h"

double tileLengthMm = 24.0 * 25.4;
double wheelRevMm = 320.0;
double motorRevMm = 1 * (36.0 / 84.0) * wheelRevMm; // how many distance travelled per motor revolution
// motorRev = travelDistance / motorRevMm
double rotateRadiusMm = 13.0 / 2 * 25.4;
//rotationRadiusMm = 
double currentLRev = 0;
double currentRRev =0;

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

        printf("Error: %.3f, velocity: %.3f, angErr: %.3f, ang: %.3f\n", error, newLinearVelocityVolt, rotateError, MJ.rotation());
        
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
    PIDClass rotatePid(0.80, 0.0005, 0, 0.7);
    //PIDClass rotatePid(0.6, 0, 0, 7.5);

    // Variables for PID
    meanVelocityPct = fabs(meanVelocityPct);
    // double maxVolt = fmin(12.0, fmax(-12.0, meanVelocityPct / 100.0 * 12.0));
    double initLeft = LeftMotors.position(rev);
    double initRight = RightMotors.position(rev);
    // Spin until done or until timeout
    while (!rotatePid.isSettled() && timeout.value() * 1000 < timeoutMs) {
      /* // Calculate error
        currentLRev = LeftMotors.position(rev) - initLeft;
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


void turnAngle(double degree, char  stop,  double centerOffsetMm, double meanVelocityPct, double timeoutMs = 3000) {
    timer timeout;
    timeout.reset();
    if (stop == 'R' ){
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
    // Calculate velocities
    double leftVelocityPct = meanVelocityPct * (leftRadius / meanRadius);
    double rightVelocityPct = meanVelocityPct * (rightRadius / meanRadius);
    // Spin motors
    LeftMotors.spinFor(motorRevLeft, rev, leftVelocityPct, velocityUnits::pct, false);
    RightMotors.spinFor(-motorRevRight, rev, rightVelocityPct, velocityUnits::pct, false);
    // Spin until done or until timeout
    while ((LeftMotors.isSpinning() || RightMotors.isSpinning()) && timeout.value() * 1000 < timeoutMs) {
        task::sleep(30);
    }
    // Stop
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
    }
    else if(stop == 'L'){
    double leftRadius = rotateRadiusMm - centerOffsetMm;
    double rightRadius = rotateRadiusMm + centerOffsetMm;
    double meanRadius = (leftRadius + rightRadius) / 2;
    // Calculate arc length (mm) to rotate
    double arcLengthLeft = (degree / 360) * (2 * M_PI * leftRadius);
    double arcLengthRight = (degree / 360) * (2 * M_PI * rightRadius);
    // Calculate motor revolutions to rotate
    double motorRevLeft = arcLengthLeft / motorRevMm;
    double motorRevRight = arcLengthRight / motorRevMm;
    // Calculate velocities
    double leftVelocityPct = meanVelocityPct * (leftRadius / meanRadius);
    double rightVelocityPct = meanVelocityPct * (rightRadius / meanRadius);
    // Spin motors
    LeftMotors.spinFor(-motorRevLeft, rev, leftVelocityPct, velocityUnits::pct, false);
    RightMotors.spinFor(+motorRevRight, rev, rightVelocityPct, velocityUnits::pct, false);
    // Spin until done or until timeout
    while ((LeftMotors.isSpinning() || RightMotors.isSpinning()) && timeout.value() * 1000 < timeoutMs) {
        task::sleep(30);
    }
    // Stop
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
    }
    
}

void intakeSetState(int state, double gg) {
    if (state == 1) {
        IntakeMotor.spin(reverse, gg, volt);
    } else if (state == -1) {
        IntakeMotor.spin(fwd, gg, volt);
    } else {
        IntakeMotor.stop(coast);
    }
}

void resetAngle(double rotation){
    MJ.setRotation(rotation, deg);
}


void autonomousggSkill() {
    //tim skills 3.0 (w/ new bot)
    // resetAngle(131);
    // resetCatapult();
    // intakeSetState(1,6);
    // driveForward(1.38 * tileLengthMm, 180, 100, 28, 1100);
    // driveForward(-0.7 * tileLengthMm, 77, 100, 97);
    // driveForward(0.38*tileLengthMm, 77, 100, 100);
    // PuncherMotors.spin(forward,12, volt);
    // //task::sleep(28000);
    resetCatapult();

    // Push alliance balls
    
    timer timeout;
    while(timeout.value() < 30 && getPunchedCount() < 45) {
        task::sleep(10);
    }
    setPuncherCanRun(false);

    // turnToAngle(131, 0, 100); 
    // driveForward(-2.4 * tileLengthMm, 90, 100, 20);
    // driveForward(-2.1 *tileLengthMm, 0, 90, 18);
    // driveForward(0.32*tileLengthMm, 45, 100, 50);
    // driveForward(-1.0*tileLengthMm, 0, 100, 60, 1000);
    // driveForward(0.405*tileLengthMm, 38, 80, 100);
    // driveForward(0.9 *tileLengthMm, 111, 30, 90);
    // wings.set(1);
    // driveForward(0.35*tileLengthMm, 111, 20, 70);
    // driveForward(1.8*tileLengthMm, 270, 40, 30);//first push with front wings
    // driveForward(-0.5*tileLengthMm, 270, 100, 100);
    // driveForward(0.8*tileLengthMm, 270, 100, 100);
    // wings.set(0);
    // driveForward(-0.5 *tileLengthMm, 180, 80, 70);
    // wings.set(1);
    // driveForward(1.3*tileLengthMm, 270, 70, 60,1600);
    //  driveForward(-0.5*tileLengthMm, 270, 100, 100);
    // driveForward(0.8*tileLengthMm, 270, 100, 100, 1000);
    // wings.set(0);
    //  driveForward(-0.4*tileLengthMm, 270, 100, 100);
    //  wings.set(1);
    // driveForward( -2 * tileLengthMm, 408, 100, 70);
    // wings.set(0);
    // driveForward(-1 * tileLengthMm, 270, 100, 100);
    // driveForward(2.1 * tileLengthMm, 360, 100, 30);
    // driveForward(-0.4 * tileLengthMm, 360, 100, 100);
    // driveForward(0.8 * tileLengthMm, 360, 100, 100);

}