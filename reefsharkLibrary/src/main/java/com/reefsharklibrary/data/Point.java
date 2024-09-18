package com.reefsharklibrary.data;

public class Point {
    private final double val;
    private final double time;


    public Point(double val, double time) {
        this.val = val;
        this.time = time;
    }

    public double getVal() {
        return val;
    }

    public double getTime() {
        return time;
    }
}
