package com.reefsharklibrary.pathType;

import com.reefsharklibrary.data.Pose2d;

import java.util.List;

public interface Path {
    List<Pose2d> generate(double resolution);

    Pose2d startPose();
    Pose2d endPose();

    double totalDistance();

    boolean isTangent(Pose2d lastPose);
}
