package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.List;

public interface RawTrajectoryInterface {
    double getAccumulatedDistance();

    double getTotalDistance();

    Pose2d getLastPose();

    List<Pose2d> getPoseList();

    void addPose(Pose2d position, double distance);

    void addSet(List<Pose2d> points, double totalDistance);

    void addTangentSet(List<Pose2d> points, double totalDistance);

    void addCallMarker(IndexCallMarker callMarker);

    void setCallMarkers(List<IndexCallMarker> callMarkers);

    void addTemporalCallMarker(TemporalCallMarker temporalCallMarker);

    void setTemporalCallMarkers(List<TemporalCallMarker> temporalCallMarkers);

    void setFollowError(Pose2d followError);

    void setEndError(Pose2d endError);

    void setEndDelay(double endDelay);

    void setTargetEndDistance(double targetEndDistance);

    TrajectoryInterface build(ConstraintSet constraints, double resolution);

}
