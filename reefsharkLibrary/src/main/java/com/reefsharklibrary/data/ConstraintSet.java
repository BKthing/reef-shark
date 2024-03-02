package com.reefsharklibrary.data;

public class ConstraintSet {

    private final VelConstraint velConstraint;
    private final AngVelConstraint angVelConstraint;

    private final Vector2d naturalDecel;

    public ConstraintSet(PIDCoeficients lateralPID, PIDCoeficients headingPID, VelConstraint velConstraint, AngVelConstraint angVelConstraint, Vector2d naturalDecel) {
        this.angVelConstraint = angVelConstraint;
        this.velConstraint = velConstraint;
        this.naturalDecel = naturalDecel;
    }


}
