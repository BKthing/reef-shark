package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.IndexCallMarker;

import java.util.List;

public interface TrajectoryInterface {

    Pose2d startPose();

    Pose2d endPose();

    void updateTargetPoint(Pose2d pose);

    Pose2d getTargetPose();

    int getTargetPoseIndex();

    Pose2d getTargetMotionState();

    boolean targetEndpoint();

    double getEndDelay();

    double getMinTime();

    Pose2d getEndError();

    List<Pose2d> poseList();

    List<IndexCallMarker> callMarkerList();
}
