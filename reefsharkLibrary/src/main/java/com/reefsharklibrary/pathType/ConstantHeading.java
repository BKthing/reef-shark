package com.reefsharklibrary.pathType;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.geometries.Geometry;

import java.util.ArrayList;
import java.util.List;

public class ConstantHeading implements Path {
    private final Geometry geometry;

    private final double heading;

    public ConstantHeading(Geometry geometry, double heading) {
        this.geometry = geometry;
        this.heading = heading;
    }

    @Override
    public List<Pose2d> generate(double resolution) {
        List<Pose2d> path = new ArrayList<>();

        for (double i = 0; i < geometry.getTotalDistance()-resolution; i += resolution) {
            path.add(geometry.getPoint(i).toPose(heading));
        }

        path.add(geometry.endPoint().toPose(heading));

        return path;
    }

    @Override
    public Pose2d startPose() {
        return geometry.startPoint().toPose(heading);
    }

    @Override
    public Pose2d endPose() {
        return geometry.endPoint().toPose(heading);
    }

    @Override
    public double totalDistance() {
        return geometry.getTotalDistance();
    }

    @Override
    public boolean isTangent(Pose2d lastPose) {
        return startPose().getHeading() == lastPose.getHeading();
    }
}
