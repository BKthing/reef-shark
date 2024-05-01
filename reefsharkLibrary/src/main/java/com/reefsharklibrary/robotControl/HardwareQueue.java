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

    public boolean updateSingle() {
        if (!hardwareActions.isEmpty()) {
            hardwareActions.remove().run();
            return true;
        } else {
            return false;
        }
    }

    public void updateAll() {
        for (HardwareAction action: hardwareActions) {
            hardwareActions.remove().run();
        }
    }

    public void update(double runTime) {
        timer.reset();

        while (!hardwareActions.isEmpty() && timer.milliSeconds()<runTime) {
            hardwareActions.remove().run();
        }
    }

    public void update(int runActions) {
        while (!hardwareActions.isEmpty() && runActions>0) {
            hardwareActions.remove().run();
            runActions--;
        }
    }

    public void update(double runTime, int maxQueueSize) {
        timer.reset();

        while (!hardwareActions.isEmpty() && timer.milliSeconds()<runTime || hardwareActions.size()>maxQueueSize) {
            hardwareActions.remove().run();
        }
    }

    public void update(int runActions, int maxQueueSize) {
        while (!hardwareActions.isEmpty() && runActions>0 || hardwareActions.size()>maxQueueSize) {
            hardwareActions.remove().run();
            runActions--;
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
