package com.reefsharklibrary.robotControl;

public class ReusableHardwareAction {
    private final HardwareQueue hardwareQueue;

    private volatile HardwareAction currentAction = () -> {};

    private boolean queued = false;

    public ReusableHardwareAction(HardwareQueue hardwareQueue) {
        this.hardwareQueue = new HardwareQueue();
    }

    public void setAction(HardwareAction hardwareAction) {
        currentAction = hardwareAction;

        //if it is currently not in a queue
        if (!queued) {
            queued = true;
            hardwareQueue.add(() -> {
                currentAction.run();
                queued = false;
            });
        }
    }

    public void setIfEmpty(HardwareAction hardwareAction) {
        if (!queued) {
            queued = true;
            currentAction = hardwareAction;
            hardwareQueue.add(() -> {
                currentAction.run();
                queued = false;
            });
        }
    }




    public boolean isQueued() {
        return queued;
    }
}
