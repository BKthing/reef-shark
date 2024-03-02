package com.reefsharklibrary.data;

public class AngVelConstraint {
    private final double maxAngAccel;
    private final double maxAngDecel;
    private final double maxAngVel;


    public AngVelConstraint(double maxAngAccel, double maxAngDecel, double maxAngVel) {
        this.maxAngAccel = maxAngAccel;
        this.maxAngDecel = maxAngDecel;
        this.maxAngVel = maxAngVel;
    }

    public AngVelConstraint(double maxAngAccel, double maxAngVel) {
        this.maxAngAccel = maxAngAccel;
        this.maxAngDecel = maxAngAccel;
        this.maxAngVel = maxAngVel;
    }

    public double getMaxAngAccel() {
        return maxAngAccel;
    }

    public double getMaxAngDecel() {
        return maxAngDecel;
    }

    public double getMaxAngVel() {
        return maxAngVel;
    }
}
