package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Point;
import com.reefsharklibrary.data.TimePose2d;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.data.MarkerExecutable;

public class TwoWheel implements DeltaFinder {

    private final double perpendicularX;
    private final double parallelY;

    private Point X, Y, deltaH;
    private Point H = new Point(0, 0);

    public TwoWheel(double perpendicularX, double parallelY) {
        this.perpendicularX = perpendicularX;
        this.parallelY = parallelY;


    }

    public void update(Point rawX, Point rawY, Point rawHeading) {

        deltaH = new Point(rawHeading.getVal() - H.getVal(), rawHeading.getTime());
        X = new Point(rawX.getVal() - perpendicularX * deltaH.getVal(), rawX.getTime());
        Y = new Point(rawY.getVal() - parallelY * deltaH.getVal(), rawY.getTime());
        H = rawHeading;

    }


    @Override
    public Point getDeltaX() {
        return X;
    }

    @Override
    public Point getDeltaY() {
        return Y;
    }

    @Override
    public Point getDeltaHeading() {
        return deltaH;
    }

    @Override
    public Point getHeading() {
        return H;
    }
}
