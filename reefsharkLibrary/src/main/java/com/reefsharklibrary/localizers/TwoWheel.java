package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.TimePose2d;

public class TwoWheel implements DeltaFinder {

    private final double perpendicularX;
    private final double parallelY;

    private TimePose2d prevPosition;
    private TimePose2d delta;
    private TimePose2d relDelta;

    public TwoWheel(double perpendicularX, double parallelY) {
        this.perpendicularX = perpendicularX;
        this.parallelY = parallelY;

        prevPosition = new TimePose2d(0, 0, 0);
    }

    public void update(TimePose2d position) {
        delta = position.elapsedTimeMinus(prevPosition);
        prevPosition = position;

        relDelta = new TimePose2d(
                delta.getX()-perpendicularX * delta.getHeading(),
                delta.getY()-parallelY * delta.getHeading(),
                delta.getHeading(),
                delta.getTime()
        );
    }

    public void update(double parallel, double perpendicular, double heading, long time) {
        update(new TimePose2d(parallel, perpendicular, heading, time));
    }

    public void update(double parallel, double perpendicular, double heading) {
        update(new TimePose2d(parallel, perpendicular, heading));
    }


    @Override
    public TimePose2d getDeltas() {
        return delta;
    }

    @Override
    public TimePose2d getRelDeltas() {
        return relDelta;
    }


}
