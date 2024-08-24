package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Rotation;

import java.util.ArrayList;
import java.util.List;

public class MecanumVelocityFinder {
    private final List<Pose2d> positions;

    private final ArrayList<Pose2d> velocities;

    private final double relMaxVel;
    private final double relMaxAccel;

    private final ConstraintSet constraints;

    public MecanumVelocityFinder(List<Pose2d> positions, ConstraintSet constraints) {
        this.positions = positions;
        this.velocities = new ArrayList<>(positions.size());
        this.constraints = constraints;
        this.relMaxVel = constraints.getMaxLinearVel()/Math.sqrt(2);

        this.relMaxAccel = constraints.getMaxLinearAccel()/Math.sqrt(2);

        assignVelocities();
    }

    private void assignVelocities() {
        velocities.add(new Pose2d(0, 0, 0));

        //TODO: figure out how to take into account the energy it takes to turn
        for (int i = 1; i<positions.size()-2; i++) {
            Pose2d difference = positions.get(i).minus(positions.get(i-1)).minimizeHeading();
            Pose2d relDifference = difference.rotateVector(positions.get(i).getHeading()+Math.PI/4);

            Pose2d relPrevVel = velocities.get(i-1).rotateVector(positions.get(i).getHeading()+Math.PI/4);

            //the amount of inches a wheel has to rotate for the turn
            //halfed bc turns are split between both sets of wheels
            double rotInches = 0;//Math.abs(constraints.getWheelBaseRadius()*difference.getHeading())/2;

            //the time it takes for each direction to travel the difference
            double xTime = (-Math.copySign(1, relDifference.getX())*relPrevVel.getX() + Math.sqrt(relPrevVel.getX()*relPrevVel.getX() + 2*(Math.abs(relDifference.getX())+rotInches)*relMaxAccel))/relMaxAccel;
            double yTime = (-Math.copySign(1, relDifference.getY())*relPrevVel.getY() + Math.sqrt(relPrevVel.getY()*relPrevVel.getY() + 2*(Math.abs(relDifference.getY())+rotInches)*relMaxAccel))/relMaxAccel;

            double time = Math.max(
                xTime,
                yTime
            );

            //checks if the time exceeds any constraints
            time = Math.max(
                    Math.max(time, Math.max(difference.getX(), difference.getY())/constraints.getMaxLinearVel()),
                    relDifference.getHeading()/constraints.getMaxAngularVel()
                    );

            if (time <= 0) throw new RuntimeException("Invalid time: " + time + " Check if constraints are valid");

            //divides the change in position by the change in time to give the velocities
            Pose2d velocity = difference.scale(1/time).getVector2d().toPose(0);

            velocity.enforceFinite();

            velocities.add(velocity);
        }

        velocities.add(new Pose2d(0, 0, 1));

        boolean decelerating = true;

        int i = velocities.size()-2;

        while (decelerating) {
            Pose2d difference = positions.get(i).minus(positions.get(i-1)).minimizeHeading();
            Pose2d relDifference = difference.rotateVector(positions.get(i).getHeading()+Math.PI/4);

            Pose2d relPrevVel = velocities.get(i+1).rotateVector(positions.get(i).getHeading()+Math.PI/4);

            //the amount of inches a wheel has to rotate for the turn
            //halfed bc turns are split between both sets of wheels
            double rotInches = 0;//Math.abs(constraints.getWheelBaseRadius()*difference.getHeading())/2;

            //the time it takes for each direction to travel the difference
            double xTime = (-Math.copySign(1, relDifference.getX())*relPrevVel.getX() + Math.sqrt(relPrevVel.getX()*relPrevVel.getX() + 2*(Math.abs(relDifference.getX())+rotInches)*relMaxAccel))/relMaxAccel;
            double yTime = (-Math.copySign(1, relDifference.getY())*relPrevVel.getY() + Math.sqrt(relPrevVel.getY()*relPrevVel.getY() + 2*(Math.abs(relDifference.getY())+rotInches)*relMaxAccel))/relMaxAccel;

            double time = Math.max(
                    xTime,
                    yTime
            );

            //checks if the time exceeds any constraints
            time = Math.max(
                    Math.max(time, Math.max(difference.getX(), difference.getY())/constraints.getMaxLinearVel()),
                    relDifference.getHeading()/constraints.getMaxAngularVel()
            );

            if (time <= 0) throw new RuntimeException("Invalid time: " + time + " Check if constraints are valid");

            //divides the change in position by the change in time to give the velocities
            Pose2d velocity = difference.scale(1/time).getVector2d().toPose(1);

            velocity.enforceFinite();

            if (i>1 && poseGreaterThan(velocities.get(i), velocity)) {
                velocities.set(i, velocity);
                i--;
            } else {
                decelerating = false;
            }

        }

    }

    private boolean poseGreaterThan(Pose2d pose1, Pose2d pose2) {
        return Math.abs(pose1.getX())>Math.abs(pose2.getX()) || Math.abs(pose1.getY())>Math.abs(pose2.getY());
    }

    public List<Pose2d> getVelocities() {
        return velocities;
    }


}
