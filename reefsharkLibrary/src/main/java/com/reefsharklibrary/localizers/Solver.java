package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Point;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.TimePose2d;

import java.util.List;

public interface Solver {
    Pose2d getRelativeFieldMovement(Point deltaX, Point deltaY, Point deltaHeading, double heading, Pose2d prevPose);
}
