package com.reefsharklibrary.pathing;

import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.List;

public class Trajectory implements TrajectoryInterface {

    private final List<Pose2d> positions;
    private final List<Pose2d> motionStates;

    private final List<IndexCallMarker> callMarkers;

    private final List<TemporalCallMarker> localTemporalMarkers;

    private final Pose2d followError;
    private final Pose2d endError;

    private int currentPoseIndex = 0;

    private final double endDelay;
    private final double minTime;
    private final int targetEndPositionThreshold;


    public Trajectory(
            List<Pose2d> positions,
            List<Pose2d> motionStates,
            List<IndexCallMarker> callMarkers,
            List<TemporalCallMarker> localTemporalMarkers,
            Pose2d followError,
            Pose2d endError,
            double endDelay,
            double minTime,
            int targetEndPositionThreshold
    ) {
        this.positions = positions;
        this.motionStates = motionStates;
        this.callMarkers = callMarkers;
        this.localTemporalMarkers = localTemporalMarkers;
        this.followError = followError;
        this.endError = endError;
        this.endDelay = endDelay;
        this.minTime = minTime;
        this.targetEndPositionThreshold = targetEndPositionThreshold;
    }

    @Override
    public Pose2d startPose() {
        return positions.get(0);
    }

    @Override
    public Pose2d endPose() {
        return positions.get(positions.size()-1);
    }

    @Override
    public void updateTargetPoint(Pose2d pose) {
        advanceForward(pose, pose.minus(positions.get(currentPoseIndex)).compareVal());
        advanceBack(pose, pose.minus(positions.get(currentPoseIndex)).compareVal());
    }

    private void advanceForward(Pose2d pose, double prevCompareVal) {
        if (currentPoseIndex < positions.size()-1) {
            double currentCompareVal = pose.minus(positions.get(currentPoseIndex + 1)).compareVal();
            if (currentCompareVal > prevCompareVal) {
                currentPoseIndex++;
                advanceForward(pose, currentCompareVal);
            }
        }
    }

    private void advanceBack(Pose2d pose, double prevCompareVal) {
        if (currentPoseIndex != 0) {
            double currentCompareVal = pose.minus(positions.get(currentPoseIndex - 1)).compareVal();
            if (currentCompareVal > prevCompareVal) {
                currentPoseIndex--;
                advanceBack(pose, currentCompareVal);
            }
        }
    }

    @Override
    public Pose2d getTargetPose() {
        return positions.get(currentPoseIndex);
    }

    @Override
    public Pose2d getTargetMotionState() {
        return  motionStates.get(currentPoseIndex);
    }

    @Override
    public boolean targetEndpoint() {
        return targetEndPositionThreshold>=currentPoseIndex;
    }

    @Override
    public double getEndDelay() {
        return endDelay;
    }

    @Override
    public double getMinTime() {
        return minTime;
    }

    @Override
    public Pose2d getEndError() {
        return endError;
    }

    @Override
    public List<Pose2d> poseList() {
        return positions;
    }

    @Override
    public List<IndexCallMarker> callMarkerList() {
        return callMarkers;
    }


}
