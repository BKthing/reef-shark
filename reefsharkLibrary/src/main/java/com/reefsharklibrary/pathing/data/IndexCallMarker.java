package com.reefsharklibrary.pathing.data;

public class IndexCallMarker {
    private final int callPosition;
    private final MarkerExecutable executable;

    public IndexCallMarker(int callPosition, MarkerExecutable executable) {
        this.callPosition = callPosition;
        this.executable = executable;
    }

    public int getCallPosition() {
        return callPosition;
    }

    public void run() {
        executable.run();
    }
}
