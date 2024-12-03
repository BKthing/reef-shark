package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.DirectionalPose;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.ArrayList;
import java.util.List;

public class Trajectory implements TrajectoryInterface {

    private final List<DirectionalPose> positions;

    private final List<Pose2d> pose2dpositions;

    private final List<IndexCallMarker> callMarkers;
    private int callMarkerIndex = 0;

    private final List<TemporalCallMarker> localTemporalMarkers;
    private int temporalMarkerIndex = 0;

    private final ElapsedTimer timer = new ElapsedTimer();

    private final Pose2d followError;
    private final Pose2d endError;

    private int currentPoseIndex = 0;

    private final double endDelay;
    private final double minTime;
    private final int targetEndPositionThreshold;

    private final double resolution;


    public Trajectory(
            List<DirectionalPose> positions,
            List<IndexCallMarker> callMarkers,
            List<TemporalCallMarker> localTemporalMarkers,
            Pose2d followError,
            Pose2d endError,
            double endDelay,
            double minTime,
            int targetEndPositionThreshold,
            double resolution
    ) {
        this.positions = positions;
        this.pose2dpositions = pose2dList();
        this.callMarkers = callMarkers;
        this.localTemporalMarkers = localTemporalMarkers;
        this.followError = followError;
        this.endError = endError;
        this.endDelay = endDelay;
        this.minTime = minTime;
        this.targetEndPositionThreshold = targetEndPositionThreshold;
        this.resolution = resolution;
    }

    private List<Pose2d> pose2dList() {
        return new ArrayList<>(positions);
    };

    @Override
    public void start() {
        timer.reset();
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
        advanceForward(pose, pose.getVector2d().minus(positions.get(currentPoseIndex).getVector2d()).compareVal());
        advanceBack(pose, pose.getVector2d().minus(positions.get(currentPoseIndex).getVector2d()).compareVal());
//        currentPoseIndex += 5;
        updateCallMarkers();
        updateTemporalMarkers();
    }

    private void advanceForward(Pose2d pose, double prevCompareVal) {
        if (currentPoseIndex < positions.size()-1) {
            double currentCompareVal = pose.getVector2d().minus(positions.get(currentPoseIndex+1).getVector2d()).compareVal();
            if (currentCompareVal < prevCompareVal) {
                currentPoseIndex++;
                advanceForward(pose, currentCompareVal);
            }
        }
    }

    private void advanceBack(Pose2d pose, double prevCompareVal) {
        if (currentPoseIndex != 0) {
            double currentCompareVal = pose.getVector2d().minus(positions.get(currentPoseIndex-1).getVector2d()).compareVal();
            if (currentCompareVal < prevCompareVal) {
                currentPoseIndex--;
                advanceBack(pose, currentCompareVal);
            }
        }
    }

    private void updateCallMarkers() {
        while (callMarkerIndex<callMarkers.size() && callMarkers.get(callMarkerIndex).callIndex(currentPoseIndex)) {
            callMarkerIndex++;
        }
    }



    private void updateTemporalMarkers() {
        while (temporalMarkerIndex<localTemporalMarkers.size() && localTemporalMarkers.get(temporalMarkerIndex).callTime(timer.seconds())) {
            temporalMarkerIndex++;
        }
    }

    @Override
    public void clearCallMarkers() {
        for (; temporalMarkerIndex<localTemporalMarkers.size(); temporalMarkerIndex++) {
            callMarkers.get(callMarkerIndex).run();
        }
    }

    @Override
    public Pose2d getTargetPose() {
        return positions.get(currentPoseIndex);
    }

    @Override
    public DirectionalPose getTargetDirectionalPose() {
        return positions.get(currentPoseIndex);
    }

    @Override
    public double getTargetDirection() {
        return positions.get(currentPoseIndex).getDirection();
    }


    @Override
    public double getForwardComponent() {
        //scales down the power as the robot gets closer to the end and as it has to turn more
        return Math.max((1-.75/(Math.pow((.06*(positions.size()-currentPoseIndex)*resolution), 4)+1))/(1+5*Math.abs(getRadiansPerInch(8))), .2);
    }

    @Override
    public double getRadiansPerInch(double lookAheadDistance) {
        int lookAhead = (int) Math.round(lookAheadDistance/resolution);
        DirectionalPose difference;

        if (currentPoseIndex<poseList().size()-1) {
            difference = poseList().get(Math.min(currentPoseIndex+lookAhead, poseList().size()-1)).minus(poseList().get(currentPoseIndex)).toDirectionalPose(Rotation.inRange(poseList().get(Math.min(currentPoseIndex+lookAhead, poseList().size()-1)).getDirection()-poseList().get(currentPoseIndex).getDirection(), Math.PI, -Math.PI));
        } else {
            difference = poseList().get(currentPoseIndex).minus(poseList().get(currentPoseIndex-1)).toDirectionalPose(Rotation.inRange(poseList().get(currentPoseIndex).getDirection()-poseList().get(currentPoseIndex-1).getDirection(), Math.PI, -Math.PI));
        }

        return difference.getDirection()/difference.getVector2d().getMagnitude();
    }

    @Override
    public double getHeadingRadiansPerInch() {
        Pose2d difference;
        if (currentPoseIndex<poseList().size()-1) {
            difference = poseList().get(currentPoseIndex+1).minus(poseList().get(currentPoseIndex));
        } else {
            difference = poseList().get(currentPoseIndex).minus(poseList().get(currentPoseIndex-1));
        }

        return difference.getHeading()/difference.getVector2d().getMagnitude();
    }

    @Override
    public int getTargetPoseIndex() {
        return currentPoseIndex;
    }

    @Override
    public boolean targetEndpoint() {
        return currentPoseIndex>=targetEndPositionThreshold;
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
        return pose2dpositions;
    }

    @Override
    public List<DirectionalPose> directionalPoseList() {
        return positions;
    }

    @Override
    public List<IndexCallMarker> callMarkerList() {
        return callMarkers;
    }

    @Override
    public int callMarkerIndex() {
        return callMarkerIndex;
    }
}
