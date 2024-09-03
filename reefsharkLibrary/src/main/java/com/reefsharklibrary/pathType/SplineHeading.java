package com.reefsharklibrary.pathType;

import com.reefsharklibrary.data.DirectionalPose;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Rotation;
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
    public List<DirectionalPose> generate(double resolution) {
        List<DirectionalPose> path = new ArrayList<>();

        Rotation heading = new Rotation();
        heading.set(startHeading-geometry.tangentAngle(0));

        double headingInterval = Rotation.inRange(endHeading-startHeading+geometry.tangentAngle(0)-geometry.tangentAngle(geometry.getTotalDistance()), Math.PI, -Math.PI)*(resolution/geometry.getTotalDistance());

//(endHeading-geometry.tangentAngle(geometry.getTotalDistance()))-(startHeading-geometry.tangentAngle(0))

        for (double i = 0; i < geometry.getTotalDistance()-resolution; i += resolution) {
            double tanAngle = geometry.tangentAngle(i);
            path.add(geometry.getPoint(i).toPose(tanAngle+heading.get()).toDirectionalPose(tanAngle));

            heading.add(headingInterval);
        }

        path.add(geometry.endPoint().toPose(endHeading).toDirectionalPose(geometry.tangentAngle(geometry.getTotalDistance())));

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
