package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;

public class test {
    ConstraintSet robot;
    TrajectorySequence test = new TrajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)), robot)
            .lineToConstantHeading(new Vector2d(5, 5))
            .build();
}
