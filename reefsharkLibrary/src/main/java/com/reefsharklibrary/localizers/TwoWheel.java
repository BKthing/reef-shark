package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Point;

public class TwoWheel implements DeltaFinder {

    private final double perpendicularX;
    private final double parallelY;

    private Point deltaX, deltaY, deltaH;
    private double prevRawX = 0, prevRawY = 0, prevRawH = 0;

    private double changeH;

    public TwoWheel(double perpendicularX, double parallelY) {
        this.perpendicularX = perpendicularX;
        this.parallelY = parallelY;


    }

    public void update(Point rawX, Point rawY, Point rawHeading) {
        changeH = rawHeading.getVal()- prevRawH;

        deltaH = new Point(deltaH.getVal()+changeH, rawHeading.getTime());
        deltaX = new Point(deltaX.getVal() + (rawX.getVal() - prevRawX - perpendicularX * changeH), rawX.getTime());
        deltaY = new Point(deltaY.getVal() + (rawY.getVal() - prevRawY - parallelY * changeH), rawY.getTime());

        prevRawX = rawX.getVal();
        prevRawY = rawY.getVal();
        prevRawH = rawHeading.getVal();
    }

    @Override
    public void clearDeltas(Point rawX, Point rawY, Point rawHeading) {
        prevRawX = rawX.getVal();
        prevRawY = rawY.getVal();
        prevRawH = rawHeading.getVal();
    }

    @Override
    public Point getDeltaX() {
        return deltaX;
    }

    @Override
    public Point getDeltaY() {
        return deltaY;
    }

    @Override
    public Point getDeltaHeading() {
        return deltaH;
    }

    @Override
    public double getChangeHeading() {
        return changeH;
    }
}
