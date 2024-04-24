package com.reefsharklibrary.robotControl;

import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.data.MarkerExecutable;

import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ConcurrentLinkedQueue;

public class HardwareQueue {
    private final ConcurrentLinkedQueue<HardwareAction> hardwareActions = new ConcurrentLinkedQueue<>();

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

    public void update(int runTime) {
        while (!hardwareActions.isEmpty() && runTime>0) {
            hardwareActions.remove().run();
            runTime--;
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
