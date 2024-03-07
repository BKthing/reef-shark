package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

public class RawTrajectory {
    private final List<Pose2d> positions = new ArrayList<>();

    private List<IndexCallMarker> callMarkers = new ArrayList<>();
    private List<TemporalCallMarker> temporalCallMarkers = new ArrayList<>();

    private Pose2d followError = new Pose2d(1, 1 ,3);
    private Pose2d endError = new Pose2d(.5, .5 ,2);

    private double endDelay = 0;
    private double minTime = 0;

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

    public Trajectory build(ConstraintSet constraints, double resolution) {
        if (temporalCallMarkers.size()>0) {
            sortTemporalMarkers();
            minTime = temporalCallMarkers.get(temporalCallMarkers.size()-1).getCallTime();
        }

        sortCallMarkers();
        indexCallMarkers(resolution);

        return new Trajectory(positions, calculateVelocities(constraints), callMarkers, temporalCallMarkers, followError, endError, endDelay, minTime);
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

    private List<Pose2d> calculateVelocities(ConstraintSet constraints) {
        List<Pose2d> velocities = new ArrayList<>();
        return velocities;
    }
}
