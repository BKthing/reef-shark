package com.reefsharklibrary.data;

import java.util.function.Function;

public class ConstraintSet {

    private final VelConstraint velConstraint;
    private final VelConstraint strafeVelConstraint;
    private final WheelBase wheelBase;
    private final AngVelConstraint angVelConstraint;

    private final Pose2d naturalDecel;

    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;

    private final double wheelBaseWidth;
    private final double wheelBaseRadius;

    private final double radiansConversion;


    public ConstraintSet(PIDCoeficients lateralPID, PIDCoeficients headingPID, VelConstraint velConstraint, VelConstraint strafeVelConstraint, AngVelConstraint angVelConstraint, Pose2d naturalDecel, double wheelBaseWidth) {
        this.angVelConstraint = angVelConstraint;
        this.velConstraint = velConstraint;
        this.strafeVelConstraint = strafeVelConstraint;
        this.wheelBase = new WheelBase(velConstraint, strafeVelConstraint);
        this.naturalDecel = naturalDecel;
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
        this.wheelBaseWidth = wheelBaseWidth;
        this.wheelBaseRadius = wheelBaseWidth/2;
        radiansConversion = (wheelBaseRadius*wheelBaseRadius)/2;
    }

    public ConstraintSet(PIDCoeficients lateralPID, PIDCoeficients headingPID, VelConstraint velConstraint, AngVelConstraint angVelConstraint, Pose2d naturalDecel, double wheelBaseWidth) {
        this.angVelConstraint = angVelConstraint;
        this.velConstraint = velConstraint;
        this.strafeVelConstraint = velConstraint;
        this.wheelBase = new WheelBase(velConstraint, strafeVelConstraint);
        this.naturalDecel = naturalDecel;
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
        this.wheelBaseWidth = wheelBaseWidth;
        this.wheelBaseRadius = wheelBaseWidth/2;
        radiansConversion = (wheelBaseRadius*wheelBaseRadius)/2;
    }

    /*
    * Returns a vector with the achievable forward and strafe values of the robot at a certain heading
    * given the max forward and strafe values
    * this assumes a linear relationship between the forward and strafe vals
    * */
    public static Vector2d getAdjustedMecanumVector(Vector2d vel, double heading) {
        //adjust heading to be in a valid range of the function
        if (heading>2*Math.PI) {
            heading = 2*Math.PI-heading;
        } else if (heading>Math.PI) {
            heading = heading-Math.PI;
        } else if (heading>Math.PI/2) {
            heading = Math.PI-heading;
        }

        double x = vel.getY()/(Math.tan(heading)+(vel.getY()/vel.getX()));

        return new Vector2d(x, -(vel.getY()/vel.getX())*(x-vel.getX()));
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

    public Pose2d getNaturalDecel() {
        return naturalDecel;
    }

    public PIDCoeficients getLateralPID() {
        return lateralPID;
    }

    public PIDCoeficients getHeadingPID() {
        return headingPID;
    }

    public double getWheelBaseWidth() {
        return wheelBaseWidth;
    }

    public double getWheelBaseRadius() {
        return wheelBaseRadius;
    }

    public double radiansToLinearVel() {
        return radiansConversion;
    }

}
