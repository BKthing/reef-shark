package com.example.reefsharklibrary.data;

public class Pose2d {
    private final int x;
    private final int y;
    private final int heading;

    public Pose2d(int x, int y, int heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public int getX() {return x;}

    public int getY() {return y;}

    public int getHeading() {return heading;}
}
