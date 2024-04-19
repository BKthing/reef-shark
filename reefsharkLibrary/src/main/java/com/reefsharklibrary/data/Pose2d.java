package com.reefsharklibrary.data;

public class Pose2d {
    private final Vector2d vector2d;
    private final double heading;

    public Pose2d(double x, double y, double heading) {
        vector2d = new Vector2d(x, y);
        this.heading = heading;
    }

    public Pose2d(Vector2d vector2d, double heading) {
        this.vector2d = vector2d;
        this.heading = heading;
    }

    public double getX() {
        return vector2d.getX();
    }

    public double getY() {
        return vector2d.getY();
    }

    public double getHeading() {
        return heading;
    }

    public double getDirection() {
        return vector2d.getDirection();
    }

    public Vector2d getVector2d() {
        return vector2d;
    }

    public Pose2d scale(double scale) {
        return new Pose2d(vector2d.scale(scale), heading*scale);
    }

    public Pose2d minus(Pose2d pose) {
        return new Pose2d(vector2d.minus(pose.getVector2d()), heading-pose.heading);
    }

    public Pose2d plus(Pose2d pose) {
        return new Pose2d(vector2d.plus(pose.getVector2d()), heading+pose.heading);
    }

    public Pose2d divide(Pose2d pose) {
        return new Pose2d(vector2d.divide(pose.getVector2d()), heading/pose.heading);
    }

    public Pose2d multiply(Pose2d pose) {
        return new Pose2d(vector2d.multiply(pose.getVector2d()), heading*pose.heading);
    }

    public Pose2d sqr() {
        return vector2d.sqr().toPose(heading*heading);
    }

    public Pose2d sqrt() {
        return vector2d.sqrt().toPose(Math.sqrt(heading));
    }

    //used to determine how close the robot is to a point
    public double compareVal() {
        return vector2d.compareVal()+Math.abs(heading*5);
    }

    public  boolean inRange(Pose2d pose) {
        return Math.abs(heading) < pose.heading && vector2d.isInRange(pose.getVector2d());
    }
}
