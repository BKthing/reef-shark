package com.reefsharklibrary.robotControl;

import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.data.MarkerExecutable;

import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

public class HardwareQueue {
    private final Queue<HardwareAction> hardwareActions = new LinkedList<>();

    private final ElapsedTimer timer;

    public HardwareQueue() {
        timer = new ElapsedTimer();
    }

    public void update(double runTime) {
        timer.reset();

        while (!hardwareActions.isEmpty() && timer.milliSeconds()<runTime) {
            hardwareActions.remove().run();
        }
    }

    public void clear() {
        hardwareActions.clear();
    }

    public boolean isEmpty() {
        return hardwareActions.isEmpty();
    }

    public int size() {
        return hardwareActions.size();
    }

    public void add(HardwareAction hardwareAction) {
        hardwareActions.add(hardwareAction);
    }


}
