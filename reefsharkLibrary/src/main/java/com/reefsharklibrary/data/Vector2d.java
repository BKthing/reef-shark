package com.reefsharklibrary.data;

public class Vector2d {
    private final double x;
    private final double y;

    public Vector2d(int x, int y) {
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
        return Math.atan(x/y);
    }
}
