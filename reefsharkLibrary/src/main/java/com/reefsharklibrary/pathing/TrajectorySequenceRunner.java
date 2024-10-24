package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.misc.ElapsedTimer;

public class TrajectorySequenceRunner {
    public enum FollowState {
        FOLLOW_TRAJECTORY,
        POINT_TURN,
        TARGET_END_POINT,
        NEXT_TRAJECTORY_DELAY,
        NO_TRAJECTORY
    }
    FollowState followState = FollowState.NO_TRAJECTORY;

    private boolean trajectoryStarted = false;

    private Pose2d targetPose;

    private TrajectorySequence trajectorySequence;
    Rotation currentRotation = new Rotation(0, 2*Math.PI);
    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;

    private final PIDLineController pidLineController;

    private final EndpointEstimator endpointEstimator;

    private final PIDPointController pidPointController;

    private final ConstraintSet constraintSet;

    private MotorPowers lastMotorPowers = new MotorPowers();

    private final ElapsedTimer trajectoryTime = new ElapsedTimer();

    private double delayTime = 0;




    public TrajectorySequenceRunner(PIDCoeficients lateralPID, PIDCoeficients headingPID, Pose2d naturalDecel, double trackWidth, ConstraintSet constraintSet) {
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
        pidLineController = new PIDLineController(lateralPID, headingPID, trackWidth, constraintSet.getLateralComponentScalar());
        endpointEstimator = new EndpointEstimator(lateralPID, headingPID, naturalDecel);
        pidPointController = new PIDPointController(lateralPID, headingPID, trackWidth);
        this.constraintSet = constraintSet;
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        this.trajectorySequence = trajectorySequence;

        trajectoryStarted = false;

//        followState = FollowState.TARGET_END_POINT;
//
//        targetPose = trajectorySequence.getCurrentTrajectory().endPose();

        followState = FollowState.FOLLOW_TRAJECTORY;
    }

    public MotorPowers update(Pose2d poseEstimate, Pose2d poseVelocity, Pose2d poseAcceleration) {
        if (!trajectoryStarted) {
            this.trajectorySequence.start();
            this.trajectorySequence.getCurrentTrajectory().start();
            trajectoryTime.reset();

            trajectoryStarted = true;
        }

        MotorPowers motorPowers = new MotorPowers();
        motorPowers.setStrafeScalar(constraintSet.getStrafeScalar());

        switch (followState) {
            case FOLLOW_TRAJECTORY -> {
                trajectorySequence.getCurrentTrajectory().updateTargetPoint(poseEstimate);
                trajectorySequence.updateGlobalTemporalMarkers();
                pidLineController.calculatePowers(poseEstimate, poseVelocity, trajectorySequence.getCurrentTrajectory().getTargetDirectionalPose(), motorPowers);
                if (trajectorySequence.getCurrentTrajectory().targetEndpoint()) {//trajectorySequence.getCurrentTrajectory().targetEndpoint()
                    targetPose = trajectorySequence.getCurrentTrajectory().endPose();
//                    followState = FollowState.NO_TRAJECTORY;

                    followState = FollowState.TARGET_END_POINT;
                }
            }
            case POINT_TURN -> {
                trajectorySequence.getCurrentTrajectory().updateTargetPoint(poseEstimate);
                trajectorySequence.updateGlobalTemporalMarkers();

                pidPointController.calculatePowers(poseEstimate, poseVelocity, trajectorySequence.getCurrentTrajectory().getTargetPose(), motorPowers);

                if (trajectorySequence.getCurrentTrajectory().targetEndpoint()) {//trajectorySequence.getCurrentTrajectory().targetEndpoint()
                    targetPose = trajectorySequence.getCurrentTrajectory().endPose();
//                    followState = FollowState.NO_TRAJECTORY;

                    followState = FollowState.TARGET_END_POINT;
                }
            }
            case TARGET_END_POINT -> {
                trajectorySequence.getCurrentTrajectory().updateTargetPoint(poseEstimate);
                trajectorySequence.updateGlobalTemporalMarkers();

                endpointEstimator.updateEndPos(poseEstimate, poseVelocity);
                pidPointController.calculatePowers(endpointEstimator.getEstimatedEndPos(), endpointEstimator.getEstimatedEndVel(), targetPose, motorPowers);

                //stops targeting endpoint if robot is close enough and has a low velocity
                if (poseEstimate.minus(targetPose).inRange(trajectorySequence.getCurrentTrajectory().getEndError()) && poseVelocity.inRange(new Pose2d(.5, .5, Math.toRadians(2)))) {
                    delayTime = Math.max(trajectorySequence.getCurrentTrajectory().getMinTime() - trajectoryTime.seconds(), trajectorySequence.getCurrentTrajectory().getEndDelay());
                    trajectoryTime.reset();
                    followState = FollowState.NEXT_TRAJECTORY_DELAY;
                }
            }
            case NEXT_TRAJECTORY_DELAY -> {
                pidPointController.calculatePowers(poseEstimate, poseVelocity, targetPose, motorPowers);
                if (trajectoryTime.seconds() > delayTime) {
                    trajectorySequence.nextTrajectory();

                    if (trajectorySequence.isFinished()) {
                        followState = FollowState.NO_TRAJECTORY;
                    } else if (trajectorySequence.getCurrentTrajectory().getClass() == Trajectory.class) {
                        trajectoryTime.reset();
                        trajectorySequence.getCurrentTrajectory().start();
                        followState = FollowState.FOLLOW_TRAJECTORY;
                    } else {
                        trajectoryTime.reset();
                        followState = FollowState.POINT_TURN;
                    }
                }
            }
        }

        lastMotorPowers = motorPowers;
        return motorPowers;
    }

    public EndpointEstimator getEndpointController() {
        return endpointEstimator;
    }

    public TrajectorySequence getTrajectorySequence() {
        return trajectorySequence;
    }

    public FollowState getFollowState() {
        return followState;
    }

    public boolean isFinished() {
        return followState == FollowState.NO_TRAJECTORY;
    }

}