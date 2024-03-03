package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;

public class TrajectoryBuilder {
    private final double resolution;

    private final Pose2d startPose;

    private final ConstraintSet constraints;

    public TrajectoryBuilder(
            Pose2d startPose,
            ConstraintSet constraints,
            double resolution
    ) {

        this.startPose = startPose;
        this.constraints = constraints;
        this.resolution = resolution;
    }


}
