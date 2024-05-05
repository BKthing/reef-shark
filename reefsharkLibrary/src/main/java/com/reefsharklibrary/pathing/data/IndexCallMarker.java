package com.reefsharklibrary.pathing.data;

public class IndexCallMarker implements Comparable<IndexCallMarker> {
    private double callDistance;
    private int callPosition;
    private final MarkerExecutable executable;

    public IndexCallMarker(double callDistance, MarkerExecutable executable) {
        this.callDistance = callDistance;
        this.executable = executable;
    }

    public void removeDistance(double distance) {
        callDistance -= distance;
    }

    public double getCallDistance() {
        return callDistance;
    }

    public void setCallPosition(int callPosition) {
        this.callPosition = callPosition;
    }

    public boolean callIndex(int index) {
        if (index >= callPosition) {
            this.run();
            return true;
        }
        return false;
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
