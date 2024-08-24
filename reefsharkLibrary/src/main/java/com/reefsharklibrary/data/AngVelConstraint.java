package com.reefsharklibrary.data;

public class AngVelConstraint {
    private final double maxAngVel;
    private final double maxAngAccel;
    private final double maxAngJerk;


    public AngVelConstraint(double maxAngVel, double maxAngAccel, double maxAngJerk) {
        this.maxAngAccel = maxAngAccel;
        this.maxAngVel = maxAngVel;
        this.maxAngJerk = maxAngJerk;
    }

    public double getMaxAngVel() {
        return maxAngVel;
    }

    public double getMaxAngAccel() {
        return maxAngAccel;
    }

    public double getMaxAngJerk() {
        return maxAngJerk;
    }
}
