package com.reefsharklibrary.data;

public class Rotation {
    private double rotation = 0;

    private final double lowerBound;
    private final double upperBound;
    private final double range;

    public Rotation(double lowerBound, double upperBound) {
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
        range = Math.abs(upperBound-lowerBound);
    }

    public double get() {
        return rotation;
    }

    private double inRange(double rotation) {
        if (rotation>=upperBound) {
             do {
                rotation -= range;
            } while (rotation>=upperBound);
        } else {
            while (rotation < lowerBound) {
                rotation += range;
            }
        }

        return rotation;
    }

    public void set(double rotation) {
        this.rotation = inRange(rotation);
    }

    public double difference(double rotationB) {
        return inRange(rotation-rotationB);
    }

    public void add(double rotationB) {
        set(rotation+rotationB);
    }

    public void subtract(double rotationB) {
        set(rotation-rotationB);
    }

}
