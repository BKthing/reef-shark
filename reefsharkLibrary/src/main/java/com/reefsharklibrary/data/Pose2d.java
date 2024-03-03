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
}
