package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.List;

public class TrajectorySequence {
    private boolean isFinished = false;
    private final List<TrajectoryInterface> trajectories;

    private int currentTrajectoryIndex = 0;

    private final List<TemporalCallMarker> globalTemporalMarkers;
    private int temporalMarkerIndex = 0;

    private final ElapsedTimer timer = new ElapsedTimer();

    public TrajectorySequence(List<TrajectoryInterface> trajectories, List<TemporalCallMarker> temporalCallMarkers) {
        if (trajectories.size() == 0) throw new NullTrajectoryExeption();

        this.trajectories = trajectories;
        globalTemporalMarkers = temporalCallMarkers;
    }

    public void start() {
        timer.reset();
    }

    public void updateGlobalTemporalMarkers() {
        while (temporalMarkerIndex<globalTemporalMarkers.size() && globalTemporalMarkers.get(temporalMarkerIndex).callTime(timer.seconds())) {
            temporalMarkerIndex++;
        }
    }

    public List<TrajectoryInterface> getTrajectories() {
        return trajectories;
    }

    public TrajectoryInterface getCurrentTrajectory() {
        return trajectories.get(currentTrajectoryIndex);
    }

    public void nextTrajectory() {
        if (currentTrajectoryIndex < trajectories.size()-1) {
            currentTrajectoryIndex++;
        } else {
            isFinished = true;
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
