package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;

public class EndpointController {

    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;

    EndpointController(PIDCoeficients lateralPID, PIDCoeficients headingPID) {
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
    }

    public MotorPowers calculatePowers(Pose2d currentPose, Pose2d currentVelocity, Pose2d targetPose) {
        return new MotorPowers();
    }
}
