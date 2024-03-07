package com.reefsharklibrary.pathing.data;

public class IndexCallMarker implements Comparable<IndexCallMarker> {
    private final double callDistance;
    private int callPosition;
    private final MarkerExecutable executable;

    public IndexCallMarker(double callDistance, MarkerExecutable executable) {
        this.callDistance = callDistance;
        this.executable = executable;
    }

    public double getCallDistance() {
        return callDistance;
    }

    public void setCallPosition(int callPosition) {
        this.callPosition = callPosition;
    }

    public int getCallPosition() {
        return callPosition;
    }

    public void run() {
        executable.run();
    }

    @Override
    public int compareTo(IndexCallMarker callMarker) {
        return Double.compare(callDistance, callMarker.callDistance);
    }
}
