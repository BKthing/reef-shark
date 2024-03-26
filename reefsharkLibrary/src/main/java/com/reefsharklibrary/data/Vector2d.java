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
        return Math.atan2(y, x);
    }

    public double getMagnitude() {
        return Math.sqrt(Math.pow(x, 2)+Math.pow(y, 2));
    }

    public Pose2d toPose(double heading) {
        return new Pose2d(this, heading);
    }

    public Vector2d scale(double scale) {
        return new Vector2d(x*scale, y*scale);
    }

    public Vector2d minus(Vector2d vector) {
        return new Vector2d(x-vector.getX(), y- vector.getY());
    }

    public Vector2d plus(Vector2d vector) {
        return new Vector2d(x+vector.getX(), y+vector.getY());
    }

    public Vector2d sqr() {
        return new Vector2d(x*x, y*y);
    }

    public Vector2d sqrt() {
        return new Vector2d(Math.sqrt(x), Math.sqrt(y));
    }

    //used to determine how close the robot is to a point
    public double compareVal() {
        return Math.abs(x)+Math.abs(y);
    }

    public boolean isInRange(Vector2d vector) {
        return Math.abs(x)<vector.x && Math.abs(y) < vector.y;
    }
}