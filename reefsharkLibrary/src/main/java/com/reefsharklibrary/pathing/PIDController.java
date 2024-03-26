package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;

public class PIDController {

    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;

    PIDController(PIDCoeficients lateralPID, PIDCoeficients headingPID) {
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
    }

    public MotorPowers calculatePowers(Pose2d currentPose, Pose2d currentVelocity, Pose2d targetPose, Pose2d targetMotionState) {
        Pose2d deltaPose = targetPose.minus(currentPose);
        return new MotorPowers();
    }
}
