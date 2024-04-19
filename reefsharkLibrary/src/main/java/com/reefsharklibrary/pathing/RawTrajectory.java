package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Vector;

public class RawTrajectory implements RawTrajectoryInterface {
    private final List<Pose2d> positions = new ArrayList<>();

    private List<IndexCallMarker> callMarkers = new ArrayList<>();
    private List<TemporalCallMarker> temporalCallMarkers = new ArrayList<>();

    private Pose2d followError = new Pose2d(1, 1 ,3);
    private Pose2d endError = new Pose2d(.5, .5 ,2);

    private double endDelay = 0;
    private double minTime = 0;

    private double targetEndDistance = 6;

    private final double initialDistance;
    private double pathDistance = 0;



    RawTrajectory(double initialDistance) {
        this.initialDistance = initialDistance;
    }

    public double getAccumulatedDistance() {
        return pathDistance;
    }

    public double getTotalDistance() {
        return initialDistance+pathDistance;
    }

    public Pose2d getLastPose() {
        return positions.get(positions.size()-1);
    }

    public void addPose(Pose2d position, double distance) {
        this.positions.add(position);
        pathDistance += distance;
    };

    //total distance is the distance the points are adding to the trajectory
    public void addSet(List<Pose2d> points, double totalDistance) {
        this.positions.addAll(points);
        this.pathDistance += totalDistance;
    }

    public void addTangentSet(List<Pose2d> points, double totalDistance) {
        //adds all but the first term to the list
        for (int i = 1; i<points.size(); i++) {
            positions.add(points.get(i));
        }

        this.pathDistance += totalDistance;
    }

    public void addCallMarker(IndexCallMarker callMarker) {
        callMarkers.add(callMarker);
    }

    public void setCallMarkers(List<IndexCallMarker> callMarkers) {
        this.callMarkers = callMarkers;
    }

    public void addTemporalCallMarker(TemporalCallMarker temporalCallMarker) {
        temporalCallMarkers.add(temporalCallMarker);
    }

    public void setTemporalCallMarkers(List<TemporalCallMarker> temporalCallMarkers) {
        this.temporalCallMarkers = temporalCallMarkers;
    }

    public void setFollowError(Pose2d followError) {
        this.followError = followError;
    }

    public void setEndError(Pose2d endError) {
        this.endError = endError;
    }

    public void setEndDelay(double endDelay) {
        this.endDelay = endDelay;
    }

    public void setTargetEndDistance(double targetEndDistance) {
        this.targetEndDistance = targetEndDistance;
    }

    public TrajectoryInterface build(ConstraintSet constraints, double resolution) {
        if (temporalCallMarkers.size()>0) {
            sortTemporalMarkers();
            minTime = temporalCallMarkers.get(temporalCallMarkers.size()-1).getCallTime();
        }

        if (callMarkers.size()>0) {
            sortCallMarkers();
            indexCallMarkers(resolution);
        }

        return new Trajectory(positions, calculateVelocities(constraints, resolution), callMarkers, temporalCallMarkers, followError, endError, endDelay, minTime, positions.size()- 1 - (int) (targetEndDistance/resolution));
    }

    private void sortTemporalMarkers() {
        Collections.sort(temporalCallMarkers);
    }

    private void sortCallMarkers() {
        Collections.sort(callMarkers);
    }

    private void indexCallMarkers(double resolution) {
        for(IndexCallMarker callMarker: callMarkers) {
            callMarker.setCallPosition(
                    //Assigns closest list position to markers
                    Math.min((int) Math.round(callMarker.getCallDistance()/resolution), positions.size())-1
            );
        }
    }


    //TODO: fix this
    private List<Pose2d> calculateVelocities(ConstraintSet constraints, double resolution) {
        List<Pose2d> velocities = new ArrayList<>();

        Pose2d prevVelocities = new Pose2d(0, 0, 0);

        prevVelocities = findVelocities(positions.get(0), prevVelocities, positions.get(0).getHeading(), constraints);
        velocities.add(prevVelocities);

        for (int i = 1; i<positions.size(); i++){

            prevVelocities = findVelocities(positions.get(i).minus(positions.get(i-1)), prevVelocities, positions.get(0).getHeading(), constraints);
            velocities.add(prevVelocities);
        }

        int i = velocities.size()-2;

        boolean decelerating = true;

        //Setting the decel point at the end
        //prevVel is set to 0's bc we want the robot to end with vel of 0's
        prevVelocities = new Pose2d(0, 0, 0);
        velocities.add(velocities.size()-1, prevVelocities);

        while (decelerating) {
            prevVelocities = findDecelVelocities(positions.get(i).minus(positions.get(i-1)), prevVelocities, constraints);

            //i<2 prevents it from calling a negative index on the next loop, should never be true on a normal trajectory
            if (poseLessThan(prevVelocities, velocities.get(i)) || i<2) {
                decelerating = false;
            } else {
                velocities.add(i, prevVelocities);
                i--;
            }
        }

        return velocities;
    }

    private boolean poseLessThan(Pose2d pose1, Pose2d pose2) {
        return pose1.getX()<pose2.getX() && pose1.getY()<pose2.getY() && pose1.getHeading()<pose2.getHeading();
    }

    private Pose2d findVelocities(Pose2d difference, Pose2d prevVelocities, double heading, ConstraintSet constraints) {
        double distance = difference.getVector2d().getMagnitude();
        double driveBaseDirection = difference.getVector2d().getDirection();
        Vector2d adjustedAccel = ConstraintSet.getAdjustedMecanumVector(constraints.getMaxLinearAccel(), heading);
        Vector2d adjustedVel = ConstraintSet.getAdjustedMecanumVector(constraints.getMaxLinearVel(), heading);

        Pose2d unScaledVelocities = new Pose2d();


        return new Pose2d(0, 0, 0);
    }

    private Pose2d findDecelVelocities(Pose2d difference, Pose2d prevVelocities, ConstraintSet constraints) {
        return new Pose2d(0, 0, 0);
    }
}
