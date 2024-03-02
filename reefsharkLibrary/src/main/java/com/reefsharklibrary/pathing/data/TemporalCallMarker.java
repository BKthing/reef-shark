package com.reefsharklibrary.pathing.data;

public class TemporalCallMarker {
    private final double callTime;
    private final MarkerExecutable executable;

    public TemporalCallMarker(int callPosition, MarkerExecutable executable) {
        this.callTime = callPosition;
        this.executable = executable;
    }

    public double getCallTime() {
        return callTime;
    }

    public void run() {
        executable.run();
    }
}
