package com.reefsharklibrary.pathType;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.geometries.Geometry;

import java.util.ArrayList;
import java.util.List;

public class LinearHeading implements Path {
    private final Geometry geometry;

    private final double startHeading;
    private final double endHeading;

    public LinearHeading(Geometry geometry, double startHeading, double endHeading) {
        this.geometry = geometry;
        this.startHeading = startHeading;
        this.endHeading = endHeading;
    }

    @Override
    public List<Pose2d> generate(double resolution) {
        List<Pose2d> path = new ArrayList<>();

        Rotation heading = new Rotation();

        heading.set(startHeading);

        double headingInterval = Rotation.inRange(endHeading-startHeading, Math.PI, -Math.PI)*(resolution/geometry.getTotalDistance());

        if (!Double.isFinite(headingInterval)) {
            throw new RuntimeException("Non finite heading interval");
        }


        for (double i = 0; i < geometry.getTotalDistance()-resolution; i += resolution) {
            if (!Double.isFinite(heading.get())) {
                throw new RuntimeException("Non finite heading");
            }

            path.add(geometry.getPoint(i).toPose(heading.get()));
            heading.add(headingInterval);
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
    public double getTangentAngle() {
        return geometry.tangentAngle(geometry.getTotalDistance());
    }

    @Override
    public boolean isTangent(double lastTangentAngle) {
        //returns true if angles are within 1 degree of eachother
        return Math.abs(geometry.tangentAngle(0)-lastTangentAngle)<Math.toRadians(1);
    }
}
