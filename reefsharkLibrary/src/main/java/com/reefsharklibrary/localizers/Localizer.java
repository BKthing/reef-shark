package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Pose2d;

public class Localizer {
    private final DeltaFinder deltaFinder;
    private final Solver solver;

    private Pose2d poseEstimate;

    public Localizer(DeltaFinder deltaFinder, Solver solver) {
        this.deltaFinder = deltaFinder;
        this.solver = solver;
    }


    public void update() {
        deltaFinder.update();
        poseEstimate = poseEstimate.plus(solver.getRelativeFieldMovement(deltaFinder.getDeltaX(), deltaFinder.getDeltaY(), deltaFinder.getDeltaHeading(), poseEstimate));

        Math.s
    }


}
