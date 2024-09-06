package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.DirectionalPose;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.ArrayList;
import java.util.List;

public class PointTurnTrajectory implements TrajectoryInterface {

    private final ElapsedTimer timer = new ElapsedTimer();

    private final List<TemporalCallMarker> localTemporalMarkers;
    private int temporalMarkerIndex = 0;

    private final Pose2d endError;
    private final double endDelay;
    private final double minTime;

    private final Pose2d startPose;

    private final Pose2d endPose;

    private Pose2d targetPoint;

    private final double turnVelocity;

    private final double turnTime;

    private final double targetEndpointAngle;


    public PointTurnTrajectory(Pose2d startPose, Pose2d endPose, double turnVelocity, List<TemporalCallMarker> localTemporalMarkers, Pose2d endError, double endDelay, double minTime, double targetEndpointAngle) {
        this.endError = endError;
        this.endDelay = endDelay;
        this.minTime = minTime;
        this.targetEndpointAngle = targetEndpointAngle;
        this.startPose = startPose;
        this.endPose = endPose;
        this.turnVelocity = Math.copySign(turnVelocity, endPose.minus(startPose).getHeading()) ;

        turnTime = (endPose.getHeading()-startPose.getHeading())/turnVelocity;

        this.localTemporalMarkers = localTemporalMarkers;

    }



    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public Pose2d startPose() {
        return startPose;
    }

    @Override
    public Pose2d endPose() {
        return endPose;
    }

    @Override
    public void updateTargetPoint(Pose2d pose) {
        if (timer.seconds()>=turnTime) {
            targetPoint = endPose;
            return;
        }

        double heading = Rotation.inRange(startPose.getHeading()+turnVelocity*timer.seconds(), 2*Math.PI, 0);

        targetPoint = new Pose2d(startPose.getVector2d(), heading);

        updateTemporalMarkers();
    }

    private void updateTemporalMarkers() {
        while (temporalMarkerIndex<localTemporalMarkers.size() && localTemporalMarkers.get(temporalMarkerIndex).callTime(timer.seconds())) {
            temporalMarkerIndex++;
        }
    }

    @Override
    public Pose2d getTargetPose() {
        return targetPoint;
    }

    @Override
    public DirectionalPose getTargetDirectionalPose() {
        return targetPoint.toDirectionalPose(0);
    }

    @Override
    public double getTargetDirection() {
        return 0;
    }

    @Override
    public int getTargetPoseIndex() {
        if (targetPoint == endPose) {
            return 1;
        } else {
            return 0;
        }
    }


    @Override
    public boolean targetEndpoint() {
        return Math.abs(Rotation.inRange(endPose.getHeading() - targetPoint.getHeading(), Math.PI, -Math.PI)) < targetEndpointAngle;
    }

    @Override
    public double getEndDelay() {
        return endDelay;
    }

    @Override
    public double getMinTime() {
        return minTime;
    }

    @Override
    public Pose2d getEndError() {
        return endError;
    }

    @Override
    public List<DirectionalPose> directionalPoseList() {
        List<DirectionalPose> poseList = new ArrayList<>();
        poseList.add(startPose.toDirectionalPose(0));
        poseList.add(endPose.toDirectionalPose(0));

        return poseList;    }

    @Override
    public List<IndexCallMarker> callMarkerList() {
        return null;
    }

    @Override
    public List<Pose2d> poseList() {
        List<Pose2d> poseList = new ArrayList<>();
        poseList.add(startPose);
        poseList.add(endPose);

        return poseList;
    }

    @Override
    public int callMarkerIndex() {
        return 0;
    }
}
