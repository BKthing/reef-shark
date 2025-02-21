package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.misc.ElapsedTimer;

public class PIDPointController {

    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;

    private double headingI = 0;

    private double lateralF = 0;

    private double headingF = 0;

    private Vector2d lateralI = new Vector2d(0, 0);

    private double lateralComponentScalar;

    private ElapsedTimer timer = new ElapsedTimer();

    private double elapsedTime;



    public PIDPointController(PIDCoeficients lateralPID, PIDCoeficients headingPID, double trackWidth) {
        this.lateralPID = lateralPID;

        //adjust the PID for the robots track width (converts radians -> circumference)
        this.headingPID = new PIDCoeficients(headingPID.getP()*trackWidth, headingPID.getI()*trackWidth, headingPID.getD()*trackWidth);
    }

    public PIDPointController(PIDCoeficients lateralPID, PIDCoeficients headingPID, double trackWidth, double lateralF, double headingF) {
        this.lateralPID = lateralPID;

        //adjust the PID for the robots track width (converts radians -> circumference)
        this.headingPID = new PIDCoeficients(headingPID.getP()*trackWidth, headingPID.getI()*trackWidth, headingPID.getD()*trackWidth);

        this.lateralF = lateralF;
        this.headingF = headingF;
    }

    //TODO: possibly change it to rotate vector, apply pid and then un-rotate so that its faster
    public void calculatePowers(Pose2d currentPose, Pose2d currentVelocity, Pose2d targetPose, MotorPowers motorPowers) {
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

        Pose2d relDistance = targetPose.minus(currentPose).rotateVector(-currentPose.getHeading());

        //added in order of importance
        motorPowers.addHeading(updateHeadingPID(Rotation.inRange(relDistance.getHeading(), Math.PI, -Math.PI), currentVelocity.getHeading()));
        motorPowers.addVector(updateLateralPID(relDistance.getVector2d(), currentVelocity.getVector2d().rotate(-currentPose.getHeading())));

    }

    public void calculatePowers(Pose2d currentPose, Pose2d currentVelocity, Pose2d targetPose, MotorPowers motorPowers, double maxLateralPower) {
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

        Pose2d relDistance = targetPose.minus(currentPose).rotateVector(-currentPose.getHeading());

        //added in order of importance
        motorPowers.addHeading(updateHeadingPID(Rotation.inRange(relDistance.getHeading(), Math.PI, -Math.PI), currentVelocity.getHeading()));

        Vector2d lateralComponent = updateLateralPID(relDistance.getVector2d(), currentVelocity.getVector2d().rotate(-currentPose.getHeading()));

        double lateralPower = Math.abs(lateralComponent.getY())+Math.abs(lateralComponent.getX());

        if (lateralPower>maxLateralPower) {
            lateralComponent.scale(maxLateralPower/lateralPower);
        }

        motorPowers.addVector(lateralComponent);

    }

    private double updateHeadingPID(double headingDiff, double headingVel) {
        headingI += headingDiff*headingPID.getI()*elapsedTime;

        return headingDiff*headingPID.getP() + headingI - headingVel*headingPID.getD() + headingF *Math.signum(headingDiff);
    }

    private Vector2d updateLateralPID(Vector2d posDiff, Vector2d posVel) {
        lateralI = lateralI.plus(posDiff.multiply(lateralPID.getI()*elapsedTime));

        return new Vector2d(
                posDiff.getX()*lateralPID.getP() + lateralI.getX() - posVel.getX()*lateralPID.getD() + lateralF *Math.signum(posDiff.getX()),
                posDiff.getY()*lateralPID.getP() + lateralI.getY() - posVel.getY()*lateralPID.getD() + lateralF *Math.signum(posDiff.getY())
        );
    }
}
