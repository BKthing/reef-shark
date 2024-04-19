package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.List;

public class RawPointTurnTrajectory implements RawTrajectoryInterface {
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
        return null;
    }

    @Override
    public void addPose(Pose2d position, double distance) {

    }

    @Override
    public void addSet(List<Pose2d> points, double totalDistance) {

    }

    @Override
    public void addTangentSet(List<Pose2d> points, double totalDistance) {

    }

    @Override
    public void addCallMarker(IndexCallMarker callMarker) {

    }

    @Override
    public void setCallMarkers(List<IndexCallMarker> callMarkers) {

    }

    @Override
    public void addTemporalCallMarker(TemporalCallMarker temporalCallMarker) {

    }

    @Override
    public void setTemporalCallMarkers(List<TemporalCallMarker> temporalCallMarkers) {

    }

    @Override
    public void setFollowError(Pose2d followError) {

    }

    @Override
    public void setEndError(Pose2d endError) {

    }

    @Override
    public void setEndDelay(double endDelay) {

    }

    @Override
    public void setTargetEndDistance(double targetEndDistance) {

    }

    @Override
    public TrajectoryInterface build(ConstraintSet constraints, double resolution) {
        return null;
    }
}
