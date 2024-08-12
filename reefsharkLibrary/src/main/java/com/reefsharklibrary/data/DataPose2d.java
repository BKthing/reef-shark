package com.reefsharklibrary.data;

public class DataPose2d extends Pose2d {

    private double distance;

    private Pose2d relVelocity;

    private Pose2d prevVel;

    public DataPose2d(double x, double y, double heading) {
        super(x, y, heading);
    }


    public double getDistance() {
        return distance;
    }

    public void setPreVel(Pose2d prevVel) {
        this.prevVel = prevVel;
    }

    public Pose2d getPrevVel() {
        return prevVel;
    }

}
