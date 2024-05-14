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
        double velAngle = targetMotionState.getDirection();
        double headingVelDiff = currentPose.getHeading()-velAngle;

        double lateralDistanceComponent = targetPose.getVector2d().minus(currentPose.getVector2d()).rotate(velAngle).getY();
        Vector2d velocityComponent = currentVelocity.getVector2d().minus(targetMotionState.getVector2d()).rotate(velAngle);
        double forwardAccelComponent = currentAcceleration.getVector2d().rotate(velAngle).getX();

        MotorPowers motorPowers = new MotorPowers();

//        motorPowers.addVector(updateLateralPID(new Vector2d(0, lateralDistanceComponent).rotate(headingVelDiff), new Vector2d(0, velocityComponent.getY()).rotate(headingVelDiff)));

        motorPowers.addVector(updateVelPID(new Vector2d(velocityComponent.getX(), 0).rotate(headingVelDiff), new Vector2d(forwardAccelComponent, 0).rotate(headingVelDiff)));

        //added in order of importance
//        motorPowers.addHeading(updateHeadingPID(adjustedPoseCorrection.getHeading(), currentVelocity.getHeading()));
//        motorPowers.addVector(updateLateralPID(adjustedPoseCorrection, currentVelocity.getVector2d().rotate(currentPose.getHeading())));
//        motorPowers.addVector(updateVelPID(velocityCorrection.getVector2d(), currentAcceleration.getVector2d()));

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

//        return new Vector2d(1, 1);
        return new Vector2d(
                velDiff.getX(),//*lateralPID.getkV() + velI.getX(),// + posAccel.getX()*lateralPID.getD(),
                velDiff.getY()//*lateralPID.getkV() + velI.getY()// + posAccel.getY()*lateralPID.getD()
        );
    };

    private Vector2d perpComponent(Pose2d input, double angle) {
        return new Vector2d(input.getVector2d().rotate(-angle).getX(), 0).rotate(angle);
    }
}
