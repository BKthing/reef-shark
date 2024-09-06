package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.TimePose2d;
import com.reefsharklibrary.data.Vector2d;

import java.util.LinkedList;
import java.util.Objects;

public class EndpointEstimator {

    private MotorPowers prevDrivePower = new MotorPowers();

    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;
    private final Pose2d naturalDecel;

    private TimePose2d estimatedEndPos;// = new TimePose2d(new Pose2d(0, 0, 0));
    private Pose2d estimatedEndVel;

    private final LinkedList<TimePose2d> prevEndPositions = new LinkedList<>();

    private double headingI = 0;

    private Vector2d lateralI = new Vector2d(0, 0);

    public EndpointEstimator(PIDCoeficients lateralPID, PIDCoeficients headingPID, Pose2d naturalDecel) {
        if (naturalDecel.getX()>0 && naturalDecel.getY()>0 && naturalDecel.getHeading()>0) {
            this.lateralPID = lateralPID;
            this.headingPID = headingPID;
            this.naturalDecel = naturalDecel;
        } else {
            throw new RuntimeException("Natural Decel <= 0");
        }
    }


    public void updateEndPos(Pose2d currentPose, Pose2d currentVelocity) {
        estimatedEndPos = new TimePose2d(estimateEndPos(currentPose, currentVelocity));
        prevEndPositions.add(estimatedEndPos);

        estimatedEndVel = estimateEndVelocity();
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
        //if the robot is not moving it will stay in the same place
        if (Velocity == 0) {
            return Position;
        }

        //making sure that the deceleration is slowing the robot down
        Deceleration *= Math.signum(Velocity);

        return Position + (Velocity*Velocity)/(2*Deceleration);

        //finding time where the robot comes to a stop
//        double t = (Acceleration+Math.sqrt(Math.pow(Acceleration, 2) - 2*Deceleration*Velocity))/-Deceleration;
        //returning estimated pos at t
//        return ((-Deceleration/3+Acceleration)*.5*t+Velocity)*t+Position;
    }

    private Pose2d estimateEndVelocity() {
        if (prevEndPositions.size()<2) {
            return new Pose2d(0, 0, 0);
        }

        if (prevEndPositions.size()>5) {
            prevEndPositions.removeFirst();
        }

        TimePose2d old = prevEndPositions.get(0);
        TimePose2d cur = prevEndPositions.get(prevEndPositions.size()-1);

        return new TimePose2d(cur.minus(old).scale((double) 1000/(cur.getTime()-old.getTime())), (cur.getTime()+old.getTime())/2);
    }

    public Pose2d getEstimatedEndPos() {
        return Objects.requireNonNullElseGet(estimatedEndPos, () -> new Pose2d(0, 0, 0));
    }

    public Pose2d getEstimatedEndVel() {
        return Objects.requireNonNullElseGet(estimatedEndVel, () -> new Pose2d(0, 0, 0));
    }
}
