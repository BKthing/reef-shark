package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.IndexCallMarker;

import java.util.List;

public interface TrajectoryInterface {

    public Pose2d startPose();

    public Pose2d endPose();

    public void updateTargetPoint(Pose2d pose);

    public Pose2d getTargetPose();

    public Pose2d getTargetMotionState();

    public boolean targetEndpoint();

    public double getEndDelay();

    public double getMinTime();

    public Pose2d getEndError();

    public List<Pose2d> poseList();

    public List<IndexCallMarker> callMarkerList();
}
