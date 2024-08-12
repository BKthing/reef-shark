package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.DataPose2d;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;

import java.util.ArrayList;
import java.util.List;

public class VelocityFinder {
    private final List<Pose2d> positions;

    private final ArrayList<Pose2d> velocities;

    private final double relMaxVel;
    private final double relMaxAccel;

    private final ConstraintSet constraints;

    public VelocityFinder(List<Pose2d> positions, ConstraintSet constraints) {
        this.positions = positions;
        this.velocities = new ArrayList<>(positions.size());
        this.constraints = constraints;
        this.relMaxVel = constraints.getMaxLinearVel().getY()/Math.sqrt(2);

        //assumes that max accel maintains a similar ratio to the ratio for max vel
        this.relMaxAccel = constraints.getMaxAccel().getY()/Math.sqrt(2);

        assignVelocities();
    }

    private void assignVelocities() {
        velocities.add(new Pose2d(0, 0, 0));

        for (int i = 1; i<positions.size()-1; i++) {
            Pose2d difference = positions.get(i).minus(positions.get(i-1));
            Pose2d relDifference = difference.rotateVector(positions.get(i).getHeading()+Math.PI/4);
            Pose2d relPrevVel = velocities.get(i-1).rotateVector(positions.get(i).getHeading()+Math.PI/4);

            //the amount of inches a wheels has to rotate for the turn
            double rotInches = Math.abs(constraints.radiansToLinearVel()*relDifference.getHeading());

            //the time it takes for each direction to travel the difference
            double xTime = (-Math.copySign(1, relDifference.getX())*relPrevVel.getX() + Math.sqrt(relPrevVel.getX()*relPrevVel.getX() + 2*relMaxAccel*(Math.abs(relDifference.getX())+rotInches)))/relMaxAccel;
            double yTime = (-Math.copySign(1, relDifference.getY())*relPrevVel.getY() + Math.sqrt(relPrevVel.getY()*relPrevVel.getY() + 2*relMaxAccel*(Math.abs(relDifference.getY())+rotInches)))/relMaxAccel;

            double time = Math.max(
                xTime,
                yTime
            );

//            if (maxVelocities.getX() < maxVelocities.getY()) {
////                if (maxVelocities.getX()>relMaxVel) {
////                    time = relDifference.getX()/relMaxVel;
////                } else {
//                    time = (maxVelocities.getX()-relPrevVel.getX())/relMaxAccel;
////                }
//            } else {
////                if (maxVelocities.getY()>relMaxVel) {
////                    time = relDifference.getY()/relMaxVel;
////                } else {
//                    time = (maxVelocities.getY()-relPrevVel.getY())/relMaxAccel;
////                }
//            }

//            double headingVel = relDifference.getHeading()/time;
//
//            double maxHeadingVel = Math.min(
//                    Math.sqrt(Math.pow(prevVel.getHeading(), 2)+2*constraints.getMaxAngularAccel()*relDifference.getHeading()),
//                    constraints.getMaxAngularVel()
//            );
//
//            if (headingVel>maxHeadingVel) {
//                time = relDifference.getHeading()/maxHeadingVel;
//            }
//
            if (time == 0) throw new RuntimeException("Time of 0");

            if (time < 0) throw new RuntimeException("Negative time");


            //divides the change in position by the change in time to give the velocity
            Pose2d velocity = difference.scale(1/time);

            velocity.enforceFinite();

            velocities.add(velocity.getVector2d().toPose(time));
        }

        velocities.add(velocities.size(), new Pose2d(0, 0, 0));
    }

    public List<Pose2d> getVelocities() {
        return velocities;
    }


}
