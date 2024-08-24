package com.reefsharklibrary.data;

public class VelConstraint {

    private final double maxVel;
    private final double maxAccel;
    private final double maxJerk;


    public VelConstraint(double maxVel, double maxAccel, double maxJerk) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.maxJerk = maxJerk;
    }

    public double getMaxVel() {
        return maxVel;
    }

    public double getMaxAccel() {
        return maxAccel;
    }

    public double getMaxJerk() {
        return maxJerk;
    }
}
