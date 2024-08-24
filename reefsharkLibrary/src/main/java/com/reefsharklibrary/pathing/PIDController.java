package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.misc.ElapsedTimer;

public class PIDController {

    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;

    private double headingI = 0;

    private Vector2d lateralI = new Vector2d(0, 0);

    private Vector2d velI = new Vector2d(0, 0);

    private double lateralComponentScalar;

    private ElapsedTimer timer = new ElapsedTimer();

    private double elapsedTime;



    PIDController(PIDCoeficients lateralPID, PIDCoeficients headingPID, double trackWidth) {
        this.lateralPID = lateralPID;

        //adjust the PID for the robots track width (converts radians -> circumference)
        this.headingPID = new PIDCoeficients(headingPID.getP()*trackWidth, headingPID.getI()*trackWidth, headingPID.getD()*trackWidth);
    }

    PIDController(PIDCoeficients lateralPID, PIDCoeficients headingPID, double trackWidth, double lateralComponentScalar) {

        this.lateralPID = lateralPID;

        //adjust the PID for the robots track width (converts radians -> circumference)
        this.headingPID = new PIDCoeficients(headingPID.getP()*trackWidth, headingPID.getI()*trackWidth, headingPID.getD()*trackWidth);

        this.lateralComponentScalar = lateralComponentScalar;
    }

    //TODO: possibly change it to rotate vector, apply pid and then un-rotate so that its faster
    public void calculatePowers(Pose2d currentPose, Pose2d currentVelocity, Pose2d currentAcceleration, Pose2d targetPose, Pose2d targetMotionState, MotorPowers motorPowers) {
//        currentPose.enforceFinite();
//        currentVelocity.enforceFinite();
//        currentAcceleration.enforceFinite();
//        targetPose.enforceFinite();
//        targetMotionState.enforceFinite();

        elapsedTime = timer.seconds();
        if (elapsedTime > .5) {
            elapsedTime = 0;
        }
        timer.reset();
        double velAngle = targetMotionState.getDirection();
        double headingVelDiff = -currentPose.getHeading()+velAngle;

        double lateralDistanceComponent = targetPose.getVector2d().minus(currentPose.getVector2d()).rotate(-velAngle).getY();
        Vector2d velocityComponent = targetMotionState.getVector2d().minus(currentVelocity.getVector2d()).rotate(-velAngle);
        double forwardAccelComponent = currentAcceleration.getVector2d().rotate(-velAngle).getX();

//        motorPowers.addVector(new Vector2d(lateralDistanceComponent/10, 0));

        //added in order of importance
        motorPowers.addHeading(updateHeadingPID(Rotation.inRange(targetPose.getHeading()-currentPose.getHeading(), Math.PI, -Math.PI), currentVelocity.getHeading()));
        motorPowers.addVector(updateLateralPID(new Vector2d(0, lateralDistanceComponent).rotate(headingVelDiff), new Vector2d(0, velocityComponent.getY()).rotate(headingVelDiff)).scale(lateralComponentScalar));
        motorPowers.addVector(new Vector2d(.6, 0).rotate(headingVelDiff));
//        motorPowers.addVector(updateVelPID(new Vector2d(velocityComponent.getX(), 0).rotate(headingVelDiff), new Vector2d(forwardAccelComponent, 0).rotate(headingVelDiff)));
    }

    private double updateHeadingPID(double headingDiff, double headingVel) {
        headingI += headingDiff*headingPID.getI()*elapsedTime;

        return -headingDiff*headingPID.getP() - headingI + headingVel*headingPID.getD();
    }

    private Vector2d updateLateralPID(Vector2d posDiff, Vector2d posVel) {
        lateralI = lateralI.plus(posDiff.multiply(lateralPID.getI()*elapsedTime));

        return new Vector2d(
                posDiff.getX()*lateralPID.getP() + lateralI.getX() + posVel.getX()*lateralPID.getD(),
                posDiff.getY()*lateralPID.getP() + lateralI.getY() + posVel.getY()*lateralPID.getD()
        );
    };

    private Vector2d updateVelPID(Vector2d velDiff, Vector2d posAccel) {
        velI = velI.plus(velDiff.multiply(lateralPID.getkP()*elapsedTime));

        return new Vector2d(
                velDiff.getX()*lateralPID.getkV() + velI.getX() + posAccel.getX()*lateralPID.getkA(),
                velDiff.getY()*lateralPID.getkV() + velI.getY() + posAccel.getY()*lateralPID.getkA()
        );
    };
}
