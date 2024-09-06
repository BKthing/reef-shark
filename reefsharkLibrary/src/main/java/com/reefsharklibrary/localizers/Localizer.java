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
        poseEstimate = poseEstimate.plus(solver.getRelativeFieldMovement(deltaFinder.getRelDeltas(), poseEstimate));
    }


}
