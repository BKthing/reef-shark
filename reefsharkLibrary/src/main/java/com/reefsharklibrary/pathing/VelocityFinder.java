package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class VelocityFinder {
    private final List<Pose2d> positions;
    private List<Pose2d> scaledPositions = new ArrayList<>();
    private List<Pose2d> maxVelocities = new ArrayList<>();
    private List<Pose2d> velocities = new ArrayList<>();


    public VelocityFinder(List<Pose2d> positions, ConstraintSet constraints) {
        this.positions = positions;

        scalePositions();

        findMaxVelocities();

        getAccelVelocities();

        applyDecelerations();
    }

    //scales each point inorder to correct for strafe and forward velocities not being the same
    //is scaled based on the robots heading
    private void scalePositions() {

    }

    //contains the highest velocities we can go at each position while still being able to cancel out bad velocities at the next position
    private void findMaxVelocities() {

    }

    public void getAccelVelocities() {
    }

    public void applyDecelerations() {

    }
}
