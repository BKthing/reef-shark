package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Pose2d;

public interface DeltaFinder {
    void update();

    Pose2d getDeltas();
}
