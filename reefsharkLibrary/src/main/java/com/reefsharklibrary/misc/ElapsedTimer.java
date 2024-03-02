package com.reefsharklibrary.misc;

public class ElapsedTimer {
    private long startTime = System.nanoTime();

    public void reset() {
        startTime = System.nanoTime();
    }

    public double nanoSeconds() {
        return System.nanoTime()-startTime;
    }

    public double seconds() {
        return nanoSeconds()/1e9;
    }

    public double milliSeconds() {
        return nanoSeconds()/1e6;
    }
}
