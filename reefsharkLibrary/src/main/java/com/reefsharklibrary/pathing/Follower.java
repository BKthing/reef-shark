package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;

interface Follower {
    void Follower(PIDCoeficients headingPID, PIDCoeficients lateralPID);
    void update(Pose2d pose);
}
