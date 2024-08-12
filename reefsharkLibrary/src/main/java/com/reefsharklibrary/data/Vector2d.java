package com.reefsharklibrary.data;

import java.util.Vector;

public class Vector2d {
    private final double x;
    private final double y;

    public Vector2d(double magnitude, double angle, boolean constructAtAngle) {
        if (constructAtAngle) {
            this.x = Math.cos(angle)*magnitude;
            this.y = Math.sin(angle)*magnitude;
        } else {
            this.x = magnitude;
            this.y = angle;
        }
    }

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

    public Vector2d scaleToMagnitude(double magnitude) {
        return new Vector2d(magnitude, this.getDirection());
    }

    public Vector2d minus(Vector2d vector) {
        return new Vector2d(x-vector.getX(), y- vector.getY());
    }

    public Vector2d plus(Vector2d vector) {
        return new Vector2d(x+vector.getX(), y+vector.getY());
    }

    public Vector2d minus(double value) {
        return new Vector2d(x-value, y- value);
    }

    public Vector2d plus(double value) {
        return new Vector2d(x+value, y+value);
    }

    public Vector2d divide(Vector2d vector) {
        return new Vector2d(x/vector.getX(), y/vector.getY());
    }

    public Vector2d multiply(Vector2d vector) {
        return new Vector2d(x*vector.getX(), y*vector.getY());
    }

    public Vector2d divide(double divisionFactor) {
        return new Vector2d(x/divisionFactor, y/divisionFactor);
    }

    public Vector2d multiply(double multiplicationFactor) {
        return new Vector2d(x*multiplicationFactor, y*multiplicationFactor);
    }

    public Vector2d sqr() {
        return new Vector2d(x*x, y*y);
    }

    public Vector2d sqrt() {
        return new Vector2d(Math.sqrt(x), Math.sqrt(y));
    }

    public Vector2d rotate(double radians) {
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);
        return new Vector2d(x*cos-y*sin, x*sin+y*cos);
    }

    //used to determine how close the robot is to a point
    public double compareVal() {
        return getMagnitude();
    }

    public boolean isInRange(Vector2d vector) {
        return Math.abs(x)<vector.x && Math.abs(y) < vector.y;
    }

    //returns 0 if it is not in a quadrant
    public int getQuadrant() {
        if (x == 0 || y == 0) {
            return 0;
        }

        if (x>0) {
            if (y>0) {
                return 1;
            } else {
                return 4;
            }
        } else {
            if (y>0) {
                return 2;
            } else {
                return 3;
            }
        }
    }

    public boolean isFinite() {
        return Double.isFinite(x) && Double.isFinite(y);
    }

    //throws and error if the vector contains non-finite numbers
    public void enforceFinite() {
        if (!isFinite()) {
            throw new RuntimeException("Invalid vector:" + this);
        }
    }

    public void enforceFinite(String string) {
        if (!isFinite()) {
            throw new RuntimeException(string);
        }
    }

    public String toString() {
        return String.format("x: %,3.2f y: %,3.2f", x, y);
    }
}