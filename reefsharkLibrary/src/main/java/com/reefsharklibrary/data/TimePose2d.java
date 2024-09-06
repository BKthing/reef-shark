package com.reefsharklibrary.data;

//A pose but it also has a time value associated with it

public class TimePose2d extends Pose2d {
    private long time;

    public TimePose2d(Pose2d pose) {
        super(pose.getX(), pose.getY(), pose.getHeading());
        this.time = System.currentTimeMillis();
    }

    public TimePose2d(Pose2d pose, long time) {
        super(pose.getX(), pose.getY(), pose.getHeading());
        this.time = time;
    }

    public TimePose2d(double x, double y, double heading) {
        super(x, y, heading);
        this.time = System.currentTimeMillis();
    }

    public TimePose2d(double x, double y, double heading, long time) {
        super(x, y, heading);
        this.time = time;
    }

    public long getTime() {
        return time;
    }

    public TimePose2d elapsedTimeMinus(TimePose2d timePose) {
        return new TimePose2d(getX()-timePose.getX(), getY()-timePose.getY(), getHeading()-timePose.getHeading(), time-timePose.getTime());
    }
}
