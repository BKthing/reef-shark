package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;

interface Localizer {
    void update();

    Pose2d getPoseEstimate();
    Pose2d getLastPoseEstimate();

    void setPoseEstimate(Pose2d pose);

    Pose2d getPoseVelocity();
    Vector2d getCentrifugalForce();

}
