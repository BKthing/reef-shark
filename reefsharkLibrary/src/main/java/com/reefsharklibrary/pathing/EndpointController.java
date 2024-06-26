package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;

import java.util.function.Function;

public class EndpointController {

    private final MotorPowers drivePower = new MotorPowers();

    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;
    private final Pose2d naturalDecel;

    private Pose2d estimatedEndPos = new Pose2d(0, 0, 0);

    private double headingI = 0;

    private Vector2d lateralI = new Vector2d(0, 0);

    EndpointController(PIDCoeficients lateralPID, PIDCoeficients headingPID, Pose2d naturalDecel) {
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
        this.naturalDecel = naturalDecel;
    }

    public MotorPowers calculatePowers(Pose2d currentPose, Pose2d currentVelocity, Pose2d targetPose) {
        drivePower.reset();
        estimatedEndPos = estimateEndPos(currentPose, currentVelocity);
//
//        drivePower.orderedAddPowers(findCorrectivePowers(targetPose.minus(estimatedEndPos)));

        return drivePower;
    }

    public MotorPowers getLastMotorPower() {
        return drivePower;
    }

    //finds motor powers to move the end point closer to the target point
    private Pose2d findCorrectivePowers(Pose2d endDiff) {
        lateralI = lateralI.plus(endDiff.getVector2d().multiply(lateralPID.getI()));
        headingI += endDiff.getHeading()*headingPID.getI();

        return new Pose2d(endDiff.getX()*lateralPID.getP() + lateralI.getX(), endDiff.getY()*lateralPID.getP() + lateralI.getY(), endDiff.getY()*headingPID.getP() + headingI);
    }

    //estimates the position where the robot will come to a stop
    private Pose2d estimateEndPos(Pose2d currentPose, Pose2d currentVelocity) {
        return new Pose2d(
                findEndPoint(naturalDecel.getX(), currentVelocity.getX(), currentPose.getX()),
                findEndPoint(naturalDecel.getY(), currentVelocity.getY(), currentPose.getY()),
                findEndPoint(naturalDecel.getHeading(), currentVelocity.getHeading(), currentPose.getHeading())
        );
    }

    private double findEndPoint(double Deceleration, double Velocity, double Position) {
        //making sure that the deceleration is slowing the robot down
        Deceleration *= Math.signum(Velocity);

        return Position + (Velocity*Velocity)/(2*Deceleration);

        //finding time where the robot comes to a stop
//        double t = (Acceleration+Math.sqrt(Math.pow(Acceleration, 2) - 2*Deceleration*Velocity))/-Deceleration;
        //returning estimated pos at t
//        return ((-Deceleration/3+Acceleration)*.5*t+Velocity)*t+Position;
    }

    public Pose2d getEstimatedEndPos() {
        return estimatedEndPos;
    }
}
