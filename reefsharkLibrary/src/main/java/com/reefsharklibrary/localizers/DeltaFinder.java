package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.TimePose2d;

public interface DeltaFinder {

    TimePose2d getDeltas();

    TimePose2d getRelDeltas();
}
