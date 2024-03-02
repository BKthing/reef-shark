package com.reefsharklibrary.data;

public class VelConstraint {
    private final double maxAccel;
    private final double maxDecel;
    private final double maxVel;


    public VelConstraint(double maxAccel, double maxDecel, double maxVel) {
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.maxVel = maxVel;
    }

    public VelConstraint(double maxAccel, double maxVel) {
        this.maxAccel = maxAccel;
        this.maxDecel = maxAccel;
        this.maxVel = maxVel;
    }

    public double getMaxAccel() {
        return maxAccel;
    }

    public double getMaxDecel() {
        return maxDecel;
    }

    public double getMaxVel() {
        return maxVel;
    }
}
