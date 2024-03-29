package com.reefsharklibrary.pathType;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.geometries.Geometry;

import java.util.ArrayList;
import java.util.List;

public class SplineHeading implements Path {
    private final Geometry geometry;

    private final double startHeading;
    private final double endHeading;

    public SplineHeading(Geometry geometry, double startHeading, double endHeading) {
        this.geometry = geometry;
        this.startHeading = startHeading;
        this.endHeading = endHeading;
    }

    @Override
    public List<Pose2d> generate(double resolution) {
        List<Pose2d> path = new ArrayList<>();

        double heading = startHeading;
        double headingInterval = (endHeading-startHeading)*resolution/geometry.getTotalDistance();

        for (double i = 0; i < geometry.getTotalDistance()-resolution; i += resolution) {
            path.add(geometry.getPoint(i).toPose(geometry.tangentAngle(i)));

            heading += headingInterval;
        }

        path.add(geometry.endPoint().toPose(endHeading));

        return path;
    }

    @Override
    public Pose2d startPose() {
        return geometry.startPoint().toPose(startHeading);
    }

    @Override
    public Pose2d endPose() {
        return geometry.endPoint().toPose(endHeading);
    }

    @Override
    public double totalDistance() {
        return geometry.getTotalDistance();
    }

    @Override
    public boolean isTangent(Pose2d lastPose) {
        return endPose() == lastPose;
    }
}
