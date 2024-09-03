package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.DirectionalPose;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.IndexCallMarker;

import java.util.List;

public interface TrajectoryInterface {

    void start();

    Pose2d startPose();

    Pose2d endPose();

    void updateTargetPoint(Pose2d pose);

    Pose2d getTargetPose();

    DirectionalPose getTargetDirectionalPose();

    double getTargetDirection();

    int getTargetPoseIndex();

    boolean targetEndpoint();

    double getEndDelay();

    double getMinTime();

    Pose2d getEndError();

    List<DirectionalPose> poseList();

    List<IndexCallMarker> callMarkerList();

    int callMarkerIndex();

}
