package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.TimePose2d;

public interface Solver {
    Pose2d getRelativeFieldMovement(TimePose2d relDeltas, Pose2d prevPose);
}
