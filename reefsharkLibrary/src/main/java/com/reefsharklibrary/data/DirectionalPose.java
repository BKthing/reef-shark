package com.reefsharklibrary.data;

public class DirectionalPose extends Pose2d {
    private final double direction;
    public DirectionalPose(double x, double y, double heading, double direction) {
        super(x, y, heading);
        this.direction = direction;
    }

    @Override
    public double getDirection() {
        return direction;
    }
}
