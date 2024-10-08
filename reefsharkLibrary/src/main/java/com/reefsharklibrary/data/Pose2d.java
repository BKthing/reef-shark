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

    public Pose2d rotateVector(double radians) {
        return new Pose2d(vector2d.rotate(radians), heading);
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

    public Pose2d minimizeHeading() {
        return vector2d.toPose(Rotation.inRange(heading, Math.PI, -Math.PI));
    }

    public Pose2d minimizeHeading(double max, double min) {
        return vector2d.toPose(Rotation.inRange(heading, max, min));
    }

    public  boolean inRange(Pose2d pose) {
        return Math.abs(heading) < pose.heading && vector2d.isInRange(pose.getVector2d());
    }

    public boolean isFinite() {
        return Double.isFinite(heading) && vector2d.isFinite();
    }

    //throws and error if the pose contains non-finite numbers
    public void enforceFinite() {
        if (!isFinite()) {
            throw new RuntimeException("Invalid pose:" + this);
        }
    }

    public void enforceFinite(String string) {
        if (!isFinite()) {
            throw new RuntimeException(string);
        }
    }

    public String toString() {
        return vector2d.toString() + String.format(" heading: %,3.2f", heading*180/Math.PI);
    }

    public String toStringRadians() {
        return vector2d.toString() + String.format(" heading: %,3.2f", heading);
    }
}
