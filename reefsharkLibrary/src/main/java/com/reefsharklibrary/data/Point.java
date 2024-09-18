package com.reefsharklibrary.data;

public class Point {
    private final double val;
    private final double time;
    private double offset;

    public Point(double val, double time) {
        this.val = val;
        this.time = time;
    }

    public double getVal() {
        return val;
    }

    public double getTime() {
        return time - offset;
    }

    public void setTimeOffset(double offset) {
        this.offset = offset;
    }
}
