package com.reefsharklibrary.robotControl;

public class ReusableHardwareAction {
    private final HardwareQueue hardwareQueue;

    private volatile boolean queued = false;

    private volatile HardwareAction currentAction = () -> queued = false;

    public ReusableHardwareAction(HardwareQueue hardwareQueue) {
        this.hardwareQueue = new HardwareQueue();
    }

    public void setAndQueueAction(HardwareAction hardwareAction) {
        setAction(hardwareAction);
        queueAction();
    }

    public void setAndQueueIfEmpty(HardwareAction hardwareAction) {
        if (!queued) {
            queued = true;
            currentAction = hardwareAction;
            hardwareQueue.add(() -> {
                currentAction.run();
                queued = false;
            });
        }
    }

    //queues the last set action
    public void queueAction() {
        //if it is currently not in a queue
        if (!queued) {
            queued = true;
            hardwareQueue.add(() -> {
                currentAction.run();
                queued = false;
            });
        }
    }

    public void setAction(HardwareAction hardwareAction) {
        currentAction = hardwareAction;
    }



    public boolean isQueued() {
        return queued;
    }
}
