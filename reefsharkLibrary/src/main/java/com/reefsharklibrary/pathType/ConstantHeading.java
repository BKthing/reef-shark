package com.reefsharklibrary.pathType;

import com.reefsharklibrary.data.DirectionalPose;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Rotation;
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

        path.add(geometry.startPoint().toPose(heading).toDirectionalPose(geometry.tangentAngle(0)));

        for (double i = resolution; i < geometry.getTotalDistance()-resolution; i += resolution) {
            path.add(geometry.getPoint(i).toPose(heading).toDirectionalPose(geometry.tangentAngle(i)));

            if ((path.get(path.size()-2).getX() == path.get(path.size()-1).getX()) && (path.get(path.size()-2).getY() == path.get(path.size()-1).getY())) {
                throw new RuntimeException("Duplicate points in path");
            }
        }

        path.add(geometry.endPoint().toPose(heading).toDirectionalPose(geometry.getTotalDistance()));

        if ((path.get(path.size()-2).getX() == path.get(path.size()-1).getX()) && (path.get(path.size()-2).getY() == path.get(path.size()-1).getY())) {
            throw new RuntimeException("Duplicate points in path");
        }

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
    public double getFirstTangentAngle() {
        return geometry.tangentAngle(0);
    }

    @Override
    public double getTangentAngle() {
        return geometry.tangentAngle(geometry.getTotalDistance());
    }

    @Override
    public boolean isTangent(double lastTangentAngle) {
        //returns true if angles are within 1 degree of eachother
        return Math.abs(Rotation.inRange(geometry.tangentAngle(0)-lastTangentAngle, Math.PI, -Math.PI))<Math.toRadians(1);
    }
}
