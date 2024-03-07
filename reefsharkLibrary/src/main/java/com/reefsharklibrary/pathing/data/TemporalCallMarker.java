package com.reefsharklibrary.pathing.data;

public class TemporalCallMarker implements Comparable<TemporalCallMarker> {
    private final double callTime;
    private final MarkerExecutable executable;

    public TemporalCallMarker(double callTime, MarkerExecutable executable) {
        this.callTime = callTime;
        this.executable = executable;
    }

    public double getCallTime() {
        return callTime;
    }

    public void run() {
        executable.run();
    }

    @Override
    public int compareTo(TemporalCallMarker temporalCallMarker) {
        return Double.compare(callTime, temporalCallMarker.getCallTime()) ;
    }
}
