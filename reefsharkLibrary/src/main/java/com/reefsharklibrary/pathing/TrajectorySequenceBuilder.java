package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.geometries.Line;
import com.reefsharklibrary.pathType.ConstantHeading;
import com.reefsharklibrary.pathType.LinearHeading;
import com.reefsharklibrary.pathType.Path;
import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.pathing.data.MarkerExecutable;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.ArrayList;
import java.util.List;

public class TrajectorySequenceBuilder {
    private final double resolution = .2;

    private final List<RawTrajectoryInterface> trajectories = new ArrayList<>();


    private final List<TemporalCallMarker> globalTemporalCallMarkers = new ArrayList<>();
    private final List<IndexCallMarker> globalCallMarkers = new ArrayList<>();




    private final ConstraintSet constraints;

    public TrajectorySequenceBuilder(
            Pose2d startPose,
            ConstraintSet constraints
    ) {
        trajectories.add(new RawTrajectory(0));
        currentTrajectory().addPose(startPose, 0);
        this.constraints = constraints;
    }

    //Path types
    public TrajectorySequenceBuilder lineToConstantHeading(Vector2d endPoint) {
        return addPath(new ConstantHeading(
                new Line(getLastPose().getVector2d(), endPoint),
                getLastPose().getHeading()) {
        });
    }

    public TrajectorySequenceBuilder lineToLineHeading(Pose2d endPoint) {
        return addPath(new LinearHeading(
                new Line(getLastPose().getVector2d(), endPoint.getVector2d()),
                getLastPose().getHeading(), endPoint.getHeading())
        );
    }



    //callMarkers
    public TrajectorySequenceBuilder callMarker(double distance, MarkerExecutable executable) {
        globalCallMarkers.add(new IndexCallMarker(distance, executable));
        return this;
    }

    public TrajectorySequenceBuilder callMarker(IndexCallMarker callMarker) {
        globalCallMarkers.add(callMarker);
        return this;
    }

    public TrajectorySequenceBuilder localCallMarker(double distance, MarkerExecutable executable) {
        currentTrajectory().addCallMarker(new IndexCallMarker(distance, executable));
        return this;
    }

    public TrajectorySequenceBuilder localCallMarker(IndexCallMarker callMarker) {
        currentTrajectory().addCallMarker(callMarker);
        return this;
    }

    //temporalMarkers
    public TrajectorySequenceBuilder temporalMarker(double time, MarkerExecutable executable) {
        globalTemporalCallMarkers.add(new TemporalCallMarker(time, executable));
        return this;
    }

    public TrajectorySequenceBuilder temporalMarker(TemporalCallMarker temporalCallMarker) {
        globalTemporalCallMarkers.add(temporalCallMarker);
        return this;
    }

    public TrajectorySequenceBuilder localTemporalMarker(double time, MarkerExecutable executable) {
        currentTrajectory().addTemporalCallMarker(new TemporalCallMarker(time, executable));
        return this;
    }

    public TrajectorySequenceBuilder localTemporalMarker(TemporalCallMarker temporalCallMarker) {
        currentTrajectory().addTemporalCallMarker(temporalCallMarker);
        return this;
    }

    public TrajectorySequenceBuilder addPath(Path pathSegment) {
        if (pathSegment.isTangent(currentTrajectory().getLastPose())) {
            currentTrajectory().addTangentSet(pathSegment.generate(resolution), pathSegment.totalDistance());
        } else {
            trajectories.add(new RawTrajectory(currentTrajectory().getTotalDistance()));
            currentTrajectory().addSet(pathSegment.generate(resolution), pathSegment.totalDistance());
        }

        return this;
    }

    public TrajectorySequenceBuilder turnTo(double heading) {
        return turn(heading);
    }

    public TrajectorySequenceBuilder turn(double turnAmount) {
        trajectories.add(new RawPointTurnTrajectory(currentTrajectory().getTotalDistance()));
        return this;
    }

    private Pose2d getLastPose() {
        return currentTrajectory().getLastPose();
    }

    public RawTrajectoryInterface currentTrajectory() {
        return trajectories.get(trajectories.size()-1);
    }

    private void sortGlobalCallMarkers() {
        for (IndexCallMarker callMarker :globalCallMarkers) {
            int i = 0;

            //sets i to the index the call marker should be placed in
            while (!(callMarker.getCallDistance() < trajectories.get(i).getTotalDistance()) && i < trajectories.size()-1) {
                i++;
            }

            if (i > 0) {
                callMarker.removeDistance(trajectories.get(i-1).getTotalDistance());
            }

            trajectories.get(i).addCallMarker(callMarker);
            globalCallMarkers.clear();
        }
    }

    public TrajectorySequence build() {
        sortGlobalCallMarkers();

        List<TrajectoryInterface> builtTrajectories = new ArrayList<>();
        for (RawTrajectoryInterface currentTrajectory: trajectories) {
            builtTrajectories.add(currentTrajectory.build(constraints, resolution));
        }

        return new TrajectorySequence(builtTrajectories, globalTemporalCallMarkers);
    }


}
