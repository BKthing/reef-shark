package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.TimePose2d;

import java.util.LinkedList;
import java.util.List;

public class CluelessTwoWheelLocalizer{
    private final double perpendicularX;
    private final double parallelY;
    private CluelessConstantAccelMath cluelessConstantAccelMath = new CluelessConstantAccelMath();
    private Pose2d poseEstimate = new Pose2d(0, 0, 0);

    private double prevRawX = 0, prevRawY = 0, prevRawH = 0;

    public LinkedList<TimePose2d> prevPositions = new LinkedList<>();
    public LinkedList<TimePose2d> prevVelocities = new LinkedList<>();
    private int maxHistorySize = 200;


    public CluelessTwoWheelLocalizer (double perpendicularX, double parallelY) {
        this.perpendicularX = perpendicularX;
        this.parallelY = parallelY;
    }

    public void update(double rawX, double rawY, double rawH, double loopTime) {
        double deltaH = rawH - prevRawH;

        double deltaX = rawX - prevRawX - perpendicularX * deltaH;
        double deltaY = rawY - prevRawY - parallelY * deltaH;

        poseEstimate = cluelessConstantAccelMath.calculate(loopTime, new Pose2d(deltaX, deltaY, deltaH), poseEstimate);

        prevPositions.add(new TimePose2d(poseEstimate));
        updatePoseVelocitiy();

        if (prevPositions.size()>maxHistorySize) {
            prevPositions.removeFirst();
        }

        if (prevVelocities.size()>maxHistorySize) {
            prevVelocities.removeFirst();
        }

    }

    private void updatePoseVelocitiy() {
        if (prevPositions.size()<2) {
            prevVelocities.add(new TimePose2d(new Pose2d(0, 0, 0)));
            return;
        }

        //looks for an index up to 4 reads ago
        int oldIndex = Math.max(0, prevPositions.size()-5);

        TimePose2d old = prevPositions.get(oldIndex);
        TimePose2d cur = prevPositions.get(prevPositions.size()-1);


        prevVelocities.add(new TimePose2d(cur.minus(old).scale( 1/(cur.getTime()-old.getTime())), (cur.getTime()+old.getTime())/2));
    }

    public Pose2d getPoseEstimate() {return poseEstimate;}

    public List<TimePose2d> getPoseHistory() {return prevPositions;}

    public void setPoseEstimate(Pose2d pose) {
        prevPositions.clear();
        prevVelocities.clear();
        prevVelocities.add(new TimePose2d(new Pose2d(0, 0, 0)));
        poseEstimate = pose;
    }

    public void clearDeltas(double rawX, double rawY, double rawH) {
        prevRawX = rawX;
        prevRawY = rawY;
        prevRawH = rawH;

    }

    public Pose2d getPoseVelocity() {return prevVelocities.get(prevVelocities.size() - 1);}

    public Pose2d getPoseAcceleration() {
        if (prevVelocities.size()<2) {
            return new Pose2d(0, 0, 0);
        }

        //looks for an index up to 4 reads ago
        int oldIndex = Math.max(0, prevVelocities.size()-5);

        TimePose2d old = prevVelocities.get(oldIndex);
        TimePose2d cur = prevVelocities.get(prevVelocities.size()-1);

        return cur.minus(old).scale(1/(cur.getTime()-old.getTime()));
    }

    public void setHistoryLimit(int maxHistorySize) {
        if (maxHistorySize<2) {
            throw new RuntimeException("History size is to small");
        }
        this.maxHistorySize = maxHistorySize+1;
    }
}
