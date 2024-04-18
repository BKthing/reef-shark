package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;

public class EndpointController {

    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;
    private final Pose2d naturalDecel;

    EndpointController(PIDCoeficients lateralPID, PIDCoeficients headingPID, Pose2d naturalDecel) {
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
        this.naturalDecel = naturalDecel;
    }

    public MotorPowers calculatePowers(Pose2d currentPose, Pose2d currentVelocity, Pose2d currentAcceleration, Pose2d targetPose) {
        Pose2d endDiff = targetPose.minus(estimateEndPos(currentPose, currentVelocity, currentAcceleration));

        return new MotorPowers();
    }

    private Pose2d estimateEndPos(Pose2d currentPose, Pose2d currentVelocity, Pose2d currentAcceleration) {

        return new Pose2d(0, 0, 0);
    }

}
