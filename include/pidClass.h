#pragma once

#include "main.h"

class PIDClass {
    public:
        PIDClass() {
            error = totalError = deltaError = 0;
            prevError = 2e18;
            kP = kI = kD = 0;
            settleErrorRange = 5;
            settleErrorMinFrames = 10;
            settledFrameCnt = 0;
        }
        PIDClass(double kProp, double kInt, double kDelta)
        : kP(kProp), kI(kInt), kD(kDelta) {
            error = totalError = deltaError = 0;
            prevError = 2e18;
            settleErrorRange = 5;
            settleErrorMinFrames = 10;
            settledFrameCnt = 0;
        }
        PIDClass(double kProp, double kInt, double kDelta, double errorRange)
        : kP(kProp), kI(kInt), kD(kDelta) {
            error = totalError = deltaError = 0;
            prevError = 2e18;
            settleErrorRange = errorRange;
            settleErrorMinFrames = 10;
            settledFrameCnt = 0;
        }
        void computeError(double newError) {
            error = newError;
            bool isCrossZero = (error > 0 && prevError < 0) || (error < 0 && prevError > 0);
            if (isCrossZero) {
                totalError = 0;
            } else {
                totalError += error;
            }
            if (prevError >= 1e18) {
                deltaError = 0;
            } else {
                deltaError = error - prevError;
            }
            prevError = error;
        }
        double getValue() {
            return error * kP + totalError * kI + deltaError * kD;
        }
        bool isSettled() {
            if (fabs(error) <= settleErrorRange) {
                if (settledFrameCnt >= settleErrorMinFrames) {
                    return true;
                } else {
                    settledFrameCnt++;
                    return false;
                }
            } else {
                settledFrameCnt = 0;
                return false;
            }
        }
    private:
        double error, totalError, deltaError, prevError;
        double kP, kI, kD; // Proportional, Integral, Delta
        double settleErrorRange;
        int settleErrorMinFrames, settledFrameCnt;
};