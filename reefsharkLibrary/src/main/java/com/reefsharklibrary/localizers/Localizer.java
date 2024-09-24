package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Point;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.TimePose2d;

import java.util.LinkedList;

public class Localizer {
    private final DeltaFinder deltaFinder;
    private final Solver solver;

    private TimePose2d poseEstimate = new TimePose2d(new Pose2d(0, 0, 0), 0);
    private int maxHistorySize = 200;

    private LinkedList<TimePose2d> prevPositions = new LinkedList<>();
    private LinkedList<TimePose2d> prevVelocities = new LinkedList<>();

    public Localizer(DeltaFinder deltaFinder, Solver solver) {
        this.deltaFinder = deltaFinder;
        this.solver = solver;
        prevVelocities.add(new TimePose2d(new Pose2d(0,0,0),0));
        prevPositions.add(new TimePose2d(new TimePose2d(0,0,0),0));
    }


    public void update(Point rawX, Point rawY, Point rawHeading) {
        deltaFinder.update(rawX, rawY, rawHeading);

        poseEstimate = new TimePose2d(poseEstimate.plus(solver.getRelativeFieldMovement(deltaFinder.getDeltaX(),
                            deltaFinder.getDeltaY(), deltaFinder.getDeltaHeading()).toPose(deltaFinder.getDeltaHeading().getVal())), (solver.getPreviousTime()));
//        poseEstimate = new TimePose2d((solver.getRelativeFieldMovement(deltaFinder.getDeltaX(),
//                deltaFinder.getDeltaY(), deltaFinder.getDeltaHeading()).toPose(deltaFinder.getHeading().getVal())), (long)(deltaFinder.getHeading().getVal()) * 1000);
        prevPositions.add(poseEstimate);
        updatePoseVelocitiy();

        if (prevPositions.size()>maxHistorySize) {
            prevPositions.removeFirst();
        }

        if (prevVelocities.size()>maxHistorySize) {
            prevVelocities.removeFirst();
        }



    }

    public LinkedList<TimePose2d> getPrevPositions() {
        return prevPositions;
    }

    public TimePose2d getVelocity() {
        return prevVelocities.get(prevVelocities.size() - 1);
    }

    public Pose2d getPoseAcceleration() {
        if (prevVelocities.size()<2) {
            return new Pose2d(0, 0, 0);
        }

        //looks for an index up to 4 reads ago
        int oldIndex = Math.max(0, prevVelocities.size()-5);

        TimePose2d old = prevVelocities.get(oldIndex);
        TimePose2d cur = prevVelocities.get(prevVelocities.size()-1);

        return cur.minus(old).scale((double) 1000/(cur.getTime()-old.getTime()));
    }

    public void setPoseEstimate(Pose2d pose) {
        prevPositions.clear();
        prevVelocities.clear();
        prevVelocities.add(new TimePose2d(new Pose2d(0, 0, 0)));
        poseEstimate = new TimePose2d(pose, 0);
        solver.setPoseEstimate(pose);
    }

    public TimePose2d getPoseEstimate() {
        return poseEstimate;
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

        prevVelocities.add(new TimePose2d(cur.minus(old).scale((double) 1000/(cur.getTime()-old.getTime())), (cur.getTime()+old.getTime())/2));
    }

    public void setHistoryLimit(int maxHistorySize) {
        if (maxHistorySize<2) {
            throw new RuntimeException("History size is to small");
        }
        this.maxHistorySize = maxHistorySize+1;
    }


}
