package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.DirectionalPose;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.data.IndexCallMarker;

import java.util.List;

public class PointTurnTrajectory implements TrajectoryInterface {

    private final ElapsedTimer timer = new ElapsedTimer();




    @Override
    public void start() {
        timer.reset();
    }

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
    public DirectionalPose getTargetDirectionalPose() {
        return null;
    }

    @Override
    public double getTargetDirection() {
        return 0;
    }

    @Override
    public int getTargetPoseIndex() {
        return 0;
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
    public List<DirectionalPose> directionalPoseList() {
        return null;
    }

    @Override
    public List<IndexCallMarker> callMarkerList() {
        return null;
    }

    @Override
    public List<Pose2d> poseList() {
        return null;
    }

    @Override
    public int callMarkerIndex() {
        return 0;
    }
}
