package com.reefsharklibrary.pathing;

import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.data.Pose2d;

import java.util.List;

public class Trajectory {

    private final List<Pose2d> positions;

    private final List<IndexCallMarker> callMarkers;

    private final Pose2d followError;

    private final Pose2d endError;


    public Trajectory(List<Pose2d> positions, List<IndexCallMarker> callMarkers, Pose2d followError, Pose2d endError) {
        this.positions = positions;
        this.callMarkers = callMarkers;
        this.followError = followError;
        this.endError = endError;
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
