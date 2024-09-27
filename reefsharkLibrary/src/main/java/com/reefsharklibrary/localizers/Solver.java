package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Point;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.TimePose2d;
import com.reefsharklibrary.data.Vector2d;

import java.util.List;

public interface Solver {
    Vector2d getRelativeFieldMovement(Point deltaX, Point deltaY, Point deltaHeading);

    double getPreviousTime();

    void setPoseEstimate(Pose2d pose);
    void setPoseEstimate(TimePose2d pose);

    void updatePoseEstimate(Pose2d pose);
    void updatePoseEstimate(TimePose2d pose);
}
