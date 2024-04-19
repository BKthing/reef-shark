package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.IndexCallMarker;

import java.util.List;

public class PointTurnTrajectory implements TrajectoryInterface {
    @Override
    public Pose2d startPose() {
        return null;
    }

    @Override
    public Pose2d endPose() {
        return null;
    }

    @Override
    public void updateTargetPoint(Pose2d pose) {

    }

    @Override
    public Pose2d getTargetPose() {
        return null;
    }

    @Override
    public Pose2d getTargetMotionState() {
        return null;
    }

    @Override
    public boolean targetEndpoint() {
        return false;
    }

    @Override
    public double getEndDelay() {
        return 0;
    }

    @Override
    public double getMinTime() {
        return 0;
    }

    @Override
    public Pose2d getEndError() {
        return null;
    }

    @Override
    public List<Pose2d> poseList() {
        return null;
    }

    @Override
    public List<IndexCallMarker> callMarkerList() {
        return null;
    }
}
