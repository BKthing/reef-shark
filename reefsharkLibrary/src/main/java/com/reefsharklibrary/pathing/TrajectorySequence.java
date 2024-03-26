package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.List;

public class TrajectorySequence {
    private boolean isFinished = false;
    private final List<Trajectory> trajectories;

    private int currentTrajectoryIndex = 0;

    private final List<TemporalCallMarker> globalTemporalMarkers;

    TrajectorySequence(List<Trajectory> trajectories, List<TemporalCallMarker> temporalCallMarkers) {
        if (trajectories.size() == 0) throw new NullTrajectoryExeption();

        this.trajectories = trajectories;
        globalTemporalMarkers = temporalCallMarkers;
    }

    public Trajectory getCurrentTrajectory() {
        return trajectories.get(currentTrajectoryIndex);
    }

    public void nextTrajectory() {
        if (currentTrajectoryIndex < trajectories.size()) {
            currentTrajectoryIndex++;
        } else {
            isFinished = false;
        }
    }

    public boolean isFinished() {
        return isFinished;
    }

    public Pose2d startPos() {
        return trajectories.get(0).startPose();
    }

    public Pose2d endPose() {
        return trajectories.get(trajectories.size()-1).endPose();
    }

}
