package com.reefsharklibrary.data;

public class Vector2d {
    private final double x;
    private final double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getDirection(){
        return Math.atan(x/y);
    }

    public double getMagnitude() {
        return Math.sqrt(Math.pow(x, 2)+Math.pow(y, 2));
    }

    public Pose2d toPose(double heading) {
        return new Pose2d(this, heading);
    }
}
