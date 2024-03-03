package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.geometries.Geometry;
import com.reefsharklibrary.geometries.Line;
import com.reefsharklibrary.pathType.ConstantHeading;
import com.reefsharklibrary.pathType.LinearHeading;
import com.reefsharklibrary.pathType.Path;

import java.util.ArrayList;
import java.util.List;

public class TrajectorySequenceBuilder {
    private final double resolution = .2;

    private List<List<Pose2d>> trajectories = new ArrayList<>();

    private List<Pose2d> currentTrajectory = new ArrayList<>();


    private final ConstraintSet constraints;

    public TrajectorySequenceBuilder(
            Pose2d startPose,
            ConstraintSet constraints
    ) {
        currentTrajectory.add(startPose);
        this.constraints = constraints;
    }

    public TrajectorySequenceBuilder lineToConstantHeading(Vector2d endPoint) {
        return addPath(new ConstantHeading(
                new Line(lastPose().getVector2d(), endPoint),
                lastPose().getHeading()) {
        });
    }

    public TrajectorySequenceBuilder lineToLineHeading(Pose2d endPoint) {
        return addPath(new LinearHeading(
                new Line(lastPose().getVector2d(), endPoint.getVector2d()),
                lastPose().getHeading(), endPoint.getHeading())
        );
    }

    public Pose2d lastPose() {
        return currentTrajectory.get(currentTrajectory.size()-1);
    }

    public TrajectorySequenceBuilder addPath(Path pathSegment) {
        currentTrajectory.addAll(pathSegment.generate(resolution));
        return this;
    }


}
