package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.List;

public class TrajectorySequence {
    private final List<Trajectory> trajectories;
    private final List<TemporalCallMarker> globalTemporalMarkers;

    TrajectorySequence(List<Trajectory> trajectories, List<TemporalCallMarker> temporalCallMarkers) {
        if (trajectories.size() == 0) throw new NullTrajectoryExeption();

        this.trajectories = trajectories;
        globalTemporalMarkers = temporalCallMarkers;
    }

    public Pose2d startPos() {
        return trajectories.get(0).startPose();
    }

    public Pose2d endPose() {
        return trajectories.get(trajectories.size()-1).endPose();
    }
}
