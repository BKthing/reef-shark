package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.geometries.CubicBezierCurve;
import com.reefsharklibrary.geometries.Line;
import com.reefsharklibrary.pathType.ConstantHeading;
import com.reefsharklibrary.pathType.LinearHeading;
import com.reefsharklibrary.pathType.Path;
import com.reefsharklibrary.pathType.SplineHeading;
import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.pathing.data.MarkerExecutable;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class TrajectorySequenceBuilder {
    private final double resolution = .15;

    private boolean firstTrajectory = true;

    private double tangentAngle;

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
        tangentAngle = startPose.getHeading();

        this.constraints = constraints;
    }

    public TrajectorySequenceBuilder setTargetEndDistance(double distance) {
        currentTrajectory().setTargetEndDistance(distance);
        return this;
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

    public TrajectorySequenceBuilder lineToSplineHeading(Pose2d endPoint) {
        return addPath(new SplineHeading(
                new Line(getLastPose().getVector2d(), endPoint.getVector2d()),
                getLastPose().getHeading(), endPoint.getHeading())
        );
    }

    public TrajectorySequenceBuilder forward(double distance) {
        double heading = getLastPose().getHeading();
        return lineToConstantHeading(new Vector2d(getLastPose().getX()+distance*Math.cos(heading), getLastPose().getY()+distance*Math.sin(heading)));
    }

    public TrajectorySequenceBuilder back(double distance) {
        double heading = getLastPose().getHeading()+Math.toRadians(180);
        return lineToConstantHeading(new Vector2d(getLastPose().getX()+distance*Math.cos(heading), getLastPose().getY()+distance*Math.sin(heading)));
    }

    public TrajectorySequenceBuilder left(double distance) {
        double heading = getLastPose().getHeading()+Math.toRadians(90);
        return lineToConstantHeading(new Vector2d(getLastPose().getX()+distance*Math.cos(heading), getLastPose().getY()+distance*Math.sin(heading)));
    }

    public TrajectorySequenceBuilder right(double distance) {
        double heading = getLastPose().getHeading()+Math.toRadians(270);
        return lineToConstantHeading(new Vector2d(getLastPose().getX()+distance*Math.cos(heading), getLastPose().getY()+distance*Math.sin(heading)));
    }


    public TrajectorySequenceBuilder splineToConstantHeading(Vector2d endPoint, double endSplineAngle) {
        return addPath(new ConstantHeading(
                new CubicBezierCurve(getLastPose().getVector2d().toPose(tangentAngle), endPoint.toPose(endSplineAngle)),
                getLastPose().getHeading()) {
        });
    }

    public TrajectorySequenceBuilder splineToLineHeading(Pose2d endPoint, double endSplineAngle) {
        return addPath(new LinearHeading(
                new CubicBezierCurve(getLastPose().getVector2d().toPose(tangentAngle), endPoint.getVector2d().toPose(endSplineAngle)),
                getLastPose().getHeading(), endPoint.getHeading())
        );
    }

    public TrajectorySequenceBuilder splineToSplineHeading(Pose2d endPoint, double endSplineAngle) {
        return addPath(new SplineHeading(
                new CubicBezierCurve(getLastPose().getVector2d().toPose(tangentAngle), endPoint.getVector2d().toPose(endSplineAngle)),
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
        if (pathSegment.isTangent(tangentAngle) || firstTrajectory) {
            currentTrajectory().addTangentSet(pathSegment.generate(resolution), pathSegment.totalDistance());
            firstTrajectory = false;
        } else {
            trajectories.add(new RawTrajectory(currentTrajectory().getTotalDistance()));
            currentTrajectory().addSet(pathSegment.generate(resolution), pathSegment.totalDistance());
        }

        tangentAngle = pathSegment.getTangentAngle();
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
        for (IndexCallMarker callMarker : globalCallMarkers) {
            int i = 0;

            //sets i to the index the call marker should be placed in
            while (callMarker.getCallDistance() > trajectories.get(i).getTotalDistance() && i < trajectories.size()-1) {
                i++;
            }

            if (i > 0) {
                callMarker.removeDistance(trajectories.get(i-1).getTotalDistance());
            }

            trajectories.get(i).addCallMarker(callMarker);
        }
        globalCallMarkers.clear();
    }

    public TrajectorySequence build() {
        sortGlobalCallMarkers();

        Collections.sort(globalTemporalCallMarkers);

        List<TrajectoryInterface> builtTrajectories = new ArrayList<>();
        for (RawTrajectoryInterface currentTrajectory: trajectories) {
            builtTrajectories.add(currentTrajectory.build(constraints, resolution));
        }

        return new TrajectorySequence(builtTrajectories, globalTemporalCallMarkers);
    }


}
