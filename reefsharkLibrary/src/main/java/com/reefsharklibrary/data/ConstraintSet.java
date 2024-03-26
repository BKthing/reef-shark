package com.reefsharklibrary.data;

public class ConstraintSet {

    private final VelConstraint velConstraint;
    private final VelConstraint strafeVelConstraint;
    private final WheelBase wheelBase;
    private final AngVelConstraint angVelConstraint;

    private final Vector2d naturalDecel;

    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;

    private final double wheelBaseWidth;
    private final double wheelBaseRadius;

    public ConstraintSet(PIDCoeficients lateralPID, PIDCoeficients headingPID, VelConstraint velConstraint, VelConstraint strafeVelConstraint, AngVelConstraint angVelConstraint, Vector2d naturalDecel, double wheelBaseWidth) {
        this.angVelConstraint = angVelConstraint;
        this.velConstraint = velConstraint;
        this.strafeVelConstraint = strafeVelConstraint;
        this.wheelBase = new WheelBase(velConstraint, strafeVelConstraint);
        this.naturalDecel = naturalDecel;
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
        this.wheelBaseWidth = wheelBaseWidth;
        this.wheelBaseRadius = wheelBaseWidth/2;
    }

    public ConstraintSet(PIDCoeficients lateralPID, PIDCoeficients headingPID, VelConstraint velConstraint, AngVelConstraint angVelConstraint, Vector2d naturalDecel, double wheelBaseWidth) {
        this.angVelConstraint = angVelConstraint;
        this.velConstraint = velConstraint;
        this.strafeVelConstraint = velConstraint;
        this.wheelBase = new WheelBase(velConstraint, strafeVelConstraint);
        this.naturalDecel = naturalDecel;
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
        this.wheelBaseWidth = wheelBaseWidth;
        this.wheelBaseRadius = wheelBaseWidth/2;
    }

    public Vector2d getMaxLinearAccel() {
        return new Vector2d(velConstraint.getMaxAccel(), strafeVelConstraint.getMaxAccel());
    }

    public Vector2d getMaxLinearDecel() {
        return new Vector2d(velConstraint.getMaxDecel(), strafeVelConstraint.getMaxDecel());
    }

    public Vector2d getMaxLinearVel() {
        return new Vector2d(velConstraint.getMaxVel(), strafeVelConstraint.getMaxVel());
    }

    public double getMaxAngularAccel() {
        return angVelConstraint.getMaxAngAccel();
    }

    public double getMaxAngularDecel() {
        return angVelConstraint.getMaxAngDecel();
    }

    public double getMaxAngularVel() {
        return angVelConstraint.getMaxAngVel();
    }

    public Pose2d getMaxAccel() {
        return getMaxLinearAccel().toPose(getMaxAngularAccel());
    }

    public Pose2d getMaxDecel() {
        return getMaxLinearDecel().toPose(getMaxAngularDecel());
    }

    public Pose2d getMaxVel() {
        return getMaxLinearVel().toPose(getMaxAngularVel());
    }

    public Vector2d getNaturalDecel() {
        return naturalDecel;
    }

    public PIDCoeficients getLateralPID() {
        return lateralPID;
    }

    public PIDCoeficients getHeadingPID() {
        return headingPID;
    }

    double getWheelBaseWidth() {
        return wheelBaseWidth;
    }

    double getWheelBaseRadius() {
        return wheelBaseRadius;
    }

}
