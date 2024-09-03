package com.reefsharklibrary.pathType;

import com.reefsharklibrary.data.DirectionalPose;
import com.reefsharklibrary.data.Pose2d;

import java.util.List;

public interface Path {
    List<DirectionalPose> generate(double resolution);

    Pose2d startPose();
    Pose2d endPose();

    double totalDistance();

    double getTangentAngle();

    boolean isTangent(double lastTangentAngle);
}
