package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Pose2d;

public interface Solver {
    Pose2d getRelativeFieldMovement(Pose2d deltas);
}
