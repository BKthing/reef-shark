package com.reefsharklibrary.data;

public class WheelBase {
    private final Vector2d wheelVectorAccel;
    private final Vector2d wheelVectorDec;
    private final Vector2d wheelVectorVel;


    public WheelBase(VelConstraint velConstraint, VelConstraint strafeVelConstraint) {
        wheelVectorAccel = new Vector2d(strafeVelConstraint.getMaxAccel(), velConstraint.getMaxAccel());
        wheelVectorDec = new Vector2d(strafeVelConstraint.getMaxDecel(), velConstraint.getMaxDecel());
        wheelVectorVel = new Vector2d(strafeVelConstraint.getMaxVel(), velConstraint.getMaxVel());
    }

    public double getMaxAccel(double heading) {
        return angleToPower(wheelVectorAccel, heading);
    }

    public double getMaxDecel(double heading) {
        return angleToPower(wheelVectorDec, heading);
    }

    public double getMaxVel(double heading) {
        return angleToPower(wheelVectorVel, heading);
    }


    private double angleToPower(Vector2d wheelVector, double heading) {
        double angle = Math.tan(heading);
        //y=(-<y>/<x>)*x+<y>
        return wheelVector.getX()+wheelVector.getY();
    }
}
