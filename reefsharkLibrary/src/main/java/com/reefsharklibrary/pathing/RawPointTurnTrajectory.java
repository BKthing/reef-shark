package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.ArrayList;
import java.util.List;

public class RawPointTurnTrajectory implements RawTrajectoryInterface {
    private final List<Pose2d> positions = new ArrayList<>();

    private List<IndexCallMarker> callMarkers = new ArrayList<>();
    private List<TemporalCallMarker> temporalCallMarkers = new ArrayList<>();

    private Pose2d followError = new Pose2d(1, 1 ,3);
    private Pose2d endError = new Pose2d(.5, .5 ,2);

    private double endDelay = 0;
    private double minTime = 0;

    private final double totalDitance;

    public RawPointTurnTrajectory(double initialDistance) {
        this.totalDitance = initialDistance;
    }

    @Override
    public double getAccumulatedDistance() {
        return 0;
    }

    @Override
    public double getTotalDistance() {
        return totalDitance;
    }

    @Override
    public Pose2d getLastPose() {
        return positions.get(positions.size()-1);
    }

    @Override
    public List<Pose2d> getPoseList() {
        return positions;
    }

    @Override
    public void addPose(Pose2d position, double distance) {
        this.positions.add(position);
    }

    @Override
    public void addSet(List<Pose2d> points, double totalDistance) {
        this.positions.addAll(points);
    }

    @Override
    public void addTangentSet(List<Pose2d> points, double totalDistance) {
        //adds all but the first term to the list
        for (int i = 1; i<points.size(); i++) {
            positions.add(points.get(i));
        }
    }

    @Override
    public void addCallMarker(IndexCallMarker callMarker) {
        callMarkers.add(callMarker);
    }

    @Override
    public void setCallMarkers(List<IndexCallMarker> callMarkers) {
        this.callMarkers = callMarkers;
    }

    @Override
    public void addTemporalCallMarker(TemporalCallMarker temporalCallMarker) {
        temporalCallMarkers.add(temporalCallMarker);
    }

    @Override
    public void setTemporalCallMarkers(List<TemporalCallMarker> temporalCallMarkers) {
        this.temporalCallMarkers = temporalCallMarkers;
    }

    @Override
    public void setFollowError(Pose2d followError) {
        this.followError = followError;
    }

    @Override
    public void setEndError(Pose2d endError) {
        this.endError = endError;
    }

    @Override
    public void setEndDelay(double endDelay) {
        this.endDelay = endDelay;
    }

    @Override
    public void setTargetEndDistance(double targetEndDistance) {

    }

    @Override
    public TrajectoryInterface build(ConstraintSet constraints, double resolution) {
        return null;
    }
}
