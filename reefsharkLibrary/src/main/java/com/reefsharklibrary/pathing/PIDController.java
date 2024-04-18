package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;

public class PIDController {

    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;

    private double headingI = 0;

    private Vector2d lateralI = new Vector2d(0, 0);

    private Vector2d velI = new Vector2d(0, 0);



    PIDController(PIDCoeficients lateralPID, PIDCoeficients headingPID, double trackWidth) {
        this.lateralPID = lateralPID;

        //adjust the PID for the robots track width (converts radians -> circumference)
        this.headingPID = new PIDCoeficients(headingPID.getP()*trackWidth, headingPID.getI()*trackWidth, headingPID.getD()*trackWidth);
    }

    //TODO: possibly change it to rotate vector, apply pid and then un-rotate so that its faster
    public MotorPowers calculatePowers(Pose2d currentPose, Pose2d currentVelocity, Pose2d currentAcceleration, Pose2d targetPose, Pose2d targetMotionState) {
        Pose2d velocityCorrection = targetMotionState.minus(currentVelocity);
        Pose2d adjustedPoseCorrection = perpComponent(targetPose.minus(currentPose), targetPose.getHeading()-currentPose.getHeading());

        MotorPowers motorPowers = new MotorPowers();

        //added in order of importance
        motorPowers.addHeading(updateHeadingPID(adjustedPoseCorrection.getHeading(), currentVelocity.getHeading()));
        motorPowers.addVector(updateLateralPID(adjustedPoseCorrection.getVector2d(), currentVelocity.getVector2d()));
        motorPowers.addVector(updateVelPID(velocityCorrection.getVector2d(), currentAcceleration.getVector2d()));

        return motorPowers;
    }

    private double updateHeadingPID(double headingDiff, double headingVel) {
        headingI += headingDiff*headingPID.getI();
        return headingDiff*headingPID.getP() + headingI + headingVel*headingPID.getD();
    }

    private Vector2d updateLateralPID(Vector2d posDiff, Vector2d posVel) {
        lateralI = lateralI.plus(posDiff.multiply(lateralPID.getI()));
        return new Vector2d(
                posDiff.getX()*lateralPID.getP() + lateralI.getX() + posVel.getX()*lateralPID.getD(),
                posDiff.getY()*lateralPID.getP() + lateralI.getY() + posVel.getY()*lateralPID.getD()
        );
    };

    private Vector2d updateVelPID(Vector2d velDiff, Vector2d posAccel) {
        velI = velI.plus(velDiff.multiply(lateralPID.getI()));
        return new Vector2d(
                velDiff.getX()*lateralPID.getP() + lateralI.getX() + posAccel.getX()*lateralPID.getD(),
                velDiff.getY()*lateralPID.getP() + lateralI.getY() + posAccel.getY()*lateralPID.getD()
        );
    };

    private Pose2d perpComponent(Pose2d input, double angle) {
        double Xr = (input.getX()*Math.cos(angle)-input.getY()*Math.sin(angle));
        return new Pose2d(Xr*Math.cos(-angle), Xr*Math.sin(-angle), input.getHeading());
    }
}
