package com.reefsharklibrary.pathType;

import com.reefsharklibrary.data.DirectionalPose;
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
    public List<DirectionalPose> generate(double resolution) {
        List<DirectionalPose> path = new ArrayList<>();

        for (double i = 0; i < geometry.getTotalDistance()-resolution; i += resolution) {
            path.add(geometry.getPoint(i).toPose(heading).toDirectionalPose(geometry.tangentAngle(i)));
        }

        path.add(geometry.endPoint().toPose(heading).toDirectionalPose(geometry.getTotalDistance()));

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
    public double getTangentAngle() {
        return geometry.tangentAngle(geometry.getTotalDistance());
    }

    @Override
    public boolean isTangent(double lastTangentAngle) {
        //returns true if angles are within 1 degree of eachother
        return Math.abs(geometry.tangentAngle(0)-lastTangentAngle)<Math.toRadians(1);
    }
}
