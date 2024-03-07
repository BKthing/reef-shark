package com.reefsharklibrary.geometries;

import com.reefsharklibrary.data.Vector2d;

import java.util.function.Function;

public class QuadraticBezierCurve implements Geometry {
    private final Vector2d startPoint;
    private final Vector2d endPoint;

    private final Function<Double, Double> x;
    private final Function<Double, Double> y;

    public QuadraticBezierCurve(Vector2d p1, Vector2d p2, Vector2d p3) {
        x = (Double d) -> Math.pow(1-d, 2)*p1.getX() + 2*(1-d)*d*p2.getX()+Math.pow(d, 2)*p3.getX();
        y = (Double d) -> Math.pow(1-d, 2)*p1.getY() + 2*(1-d)*d*p2.getY()+Math.pow(d, 2)*p3.getY();

        startPoint = p1;
        endPoint = p3;
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
