package com.reefsharklibrary.data;

import java.util.function.Function;

public class ConstraintSet {

    private final VelConstraint velConstraint;
    private final AngVelConstraint angVelConstraint;

    private final Pose2d naturalDecel;

    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;

    private final double wheelBaseRadius;





    public ConstraintSet(PIDCoeficients lateralPID, PIDCoeficients headingPID, VelConstraint velConstraint, AngVelConstraint angVelConstraint, Pose2d naturalDecel, double wheelBaseRadius) {
        this.angVelConstraint = angVelConstraint;
        this.velConstraint = velConstraint;
        this.naturalDecel = naturalDecel;
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
        this.wheelBaseRadius = wheelBaseRadius;
    }

    /*
    * Returns a vector with the achievable forward and strafe values of the robot at a certain heading
    * given the max forward and strafe values
    * this assumes a linear relationship between the forward and strafe vals
    * */
    public static Vector2d getAdjustedMecanumVector(Vector2d vel, double heading) {
        if (heading<0 || heading>2*Math.PI) throw new RuntimeException("heading out of bounds: " + heading);
        //adjust heading to be in a valid range of the function
        if (heading>3*Math.PI/2) {
            heading = 2*Math.PI-heading;
        } else if (heading>Math.PI) {
            heading = heading-Math.PI;
        } else if (heading>Math.PI/2) {
            heading = Math.PI-heading;
        }

        if (heading<0 || heading>Math.PI/2) throw new RuntimeException("heading out of bounds");

        double x = vel.getY()/(Math.tan(heading)+(vel.getY()/vel.getX()));

        return new Vector2d(x, -(vel.getY()/vel.getX())*(x-vel.getX()));
    }

    public double getMaxLinearVel() {
        return velConstraint.getMaxVel();
    }

    public double getMaxLinearAccel() {
        return velConstraint.getMaxAccel();
    }

    public double getMaxLinearJerk() {
        return velConstraint.getMaxJerk();
    }


    public double getMaxAngularVel() {
        return angVelConstraint.getMaxAngVel();
    }

    public double getMaxAngularAccel() {
        return angVelConstraint.getMaxAngAccel();
    }

    public double getMaxAngularJerk() {
        return angVelConstraint.getMaxAngJerk();
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

    public double getWheelBaseRadius() {
        return wheelBaseRadius;
    }

}
