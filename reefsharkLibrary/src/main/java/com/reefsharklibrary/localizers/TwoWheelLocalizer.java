package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;

class TwoWheelLocalizer implements Localizer {
    private Pose2d poseEstimate = new Pose2d(0, 0, 0);
    @Override
    public void update() {

    }

    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public Pose2d getLastPoseEstimate() {
        return null;
    }

    @Override
    public void setPoseEstimate(Pose2d pose) {
        poseEstimate = pose;
    }

    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public Vector2d getCentrifugalForce() {
        return null;
    }
}
