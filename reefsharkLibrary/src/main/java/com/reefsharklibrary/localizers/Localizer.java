package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.TimePose2d;
import com.reefsharklibrary.data.Vector2d;

import java.util.List;

public interface Localizer {
    void update();

    Pose2d getPoseEstimate();
    List<TimePose2d> getPoseHistory();

    void setPoseEstimate(Pose2d pose);

    Pose2d getPoseVelocity();
    Pose2d getPoseAcceleration();

    void setHistoryLimit(int maxHistorySize);

}
