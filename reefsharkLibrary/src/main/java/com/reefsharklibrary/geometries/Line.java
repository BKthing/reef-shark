package com.reefsharklibrary.geometries;

import com.reefsharklibrary.data.Vector2d;

import java.util.function.Function;

public class Line implements Geometry {
    private final Vector2d startPoint;
    private final Vector2d endPoint;
    private final Vector2d difference;
    private final double distance;
    private final double angle;
    private final Function<Double, Double> x;
    private final Function<Double, Double> y;

    public Line(Vector2d startPoint, Vector2d endPoint) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        difference = new Vector2d(startPoint.getX()-endPoint.getX(), startPoint.getY()-endPoint.getY());
        distance = difference.getMagnitude();
        angle = difference.getDirection();
        x = (Double d) -> d*Math.cos(angle)+ startPoint.getX();
        y = (Double d) -> d*Math.sin(angle)+ startPoint.getY();
    }

    @Override
    public Vector2d getPoint(double distance) {
        return new Vector2d(x.apply(distance), y.apply(distance));
    }

    @Override
    public double tangentAngle(double distance) {
        return angle;
    }

    @Override
    public double getTotalDistance() {
        return distance;
    }

    @Override
    public Vector2d startPoint() {
        return startPoint;
    }

    @Override
    public Vector2d endPoint() {
        return endPoint;
    }
}
