package com.reefsharklibrary.pathing;

import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.List;

public class Trajectory {

    private final List<Pose2d> positions;
    private final List<Pose2d> motionStates;

    private final List<IndexCallMarker> callMarkers;

    private final List<TemporalCallMarker> localTemporalMarkers;

    private final Pose2d followError;
    private final Pose2d endError;

    private final double endDelay;
    private final double minTime;


    public Trajectory(
            List<Pose2d> positions,
            List<Pose2d> motionStates,
            List<IndexCallMarker> callMarkers,
            List<TemporalCallMarker> localTemporalMarkers,
            Pose2d followError,
            Pose2d endError,
            double endDelay,
            double minTime
    ) {
        this.positions = positions;
        this.motionStates = motionStates;
        this.callMarkers = callMarkers;
        this.localTemporalMarkers = localTemporalMarkers;
        this.followError = followError;
        this.endError = endError;
        this.endDelay = endDelay;
        this.minTime = minTime;
    }

    public Pose2d startPose() {
        return positions.get(0);
    }

    public Pose2d endPose() {
        return positions.get(positions.size()-1);
    }

    public List<Pose2d> poseList() {
        return positions;
    }

    public List<IndexCallMarker> callMarkerList() {
        return callMarkers;
    }


}
