package com.reefsharklibrary.geometries;

import com.reefsharklibrary.data.Vector2d;

import java.util.function.Function;

public class CubicBezierCurve implements Geometry {
    private final Vector2d startPoint;
    private final Vector2d endPoint;
    private final Vector2d difference;
    private final double distance;
    private final double angle;
    private final Function<Double, Double> x;
    private final Function<Double, Double> y;

    public CubicBezierCurve(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4) {
        startPoint = p1;
        endPoint = p4;

        x = (Double d) -> Math.pow(1-d, 3)*p1.getX() + 3*Math.pow(1-d, 2)*d*p2.getX() + 3*(d-1)*Math.pow(d, 2)*p3.getX() + Math.pow(d, 3)*p4.getX();
        y = (Double d) -> Math.pow(1-d, 3)*p1.getY() + 3*Math.pow(1-d, 2)*d*p2.getY() + 3*(d-1)*Math.pow(d, 2)*p3.getY() + Math.pow(d, 3)*p4.getY();

    }

    @Override
    public Vector2d getPoint(double distance) {
        return null;
    }

    @Override
    public double tangentAngle(double distance) {
        return 0;
    }

    @Override
    public double getTotalDistance() {
        return 0;
    }

    @Override
    public Vector2d startPoint() {
        return null;
    }

    @Override
    public Vector2d endPoint() {
        return null;
    }
}
