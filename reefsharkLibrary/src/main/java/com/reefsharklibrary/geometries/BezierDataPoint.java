package com.reefsharklibrary.geometries;

import com.reefsharklibrary.data.Vector2d;

public class BezierDataPoint {
    private final Vector2d vector2d;
    private final double distance;
    private final double t;

    //only used for the first point of a list
    public BezierDataPoint(Vector2d vector2d) {
        this.vector2d = vector2d;

        distance = 0;
        t = 0;
    }

    public BezierDataPoint(Vector2d vector2d, BezierDataPoint prevPoint, double t) {
        this.vector2d = vector2d;

        distance = prevPoint.getDistance()+vector2d.minus(prevPoint.getVector2d()).getMagnitude();
        this.t = t;
    }

    public Vector2d getVector2d() {
        return vector2d;
    }

    public double getDistance() {
        return distance;
    }

    public double getT() {
        return t;
    }
}
