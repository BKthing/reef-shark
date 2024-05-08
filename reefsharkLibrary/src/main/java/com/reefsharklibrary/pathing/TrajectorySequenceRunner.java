package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.localizers.Localizer;
import com.reefsharklibrary.misc.ElapsedTimer;

import java.util.List;

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

    private final PIDController pidController;

    private final EndpointController endpointController;

    private MotorPowers lastMotorPowers = new MotorPowers();

    private final ElapsedTimer trajectoryTime = new ElapsedTimer();
    private double delayTime = 0;




    public TrajectorySequenceRunner(PIDCoeficients lateralPID, PIDCoeficients headingPID, Pose2d naturalDecel, double trackWidth) {
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
        pidController = new PIDController(lateralPID, headingPID, trackWidth);
        endpointController = new EndpointController(lateralPID, headingPID, naturalDecel);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        this.trajectorySequence = trajectorySequence;

        trajectoryStarted = false;

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

        switch (followState) {
            case FOLLOW_TRAJECTORY:
                trajectorySequence.getCurrentTrajectory().updateTargetPoint(poseEstimate);
                trajectorySequence.updateGlobalTemporalMarkers();
//                motorPowers = pidController.calculatePowers(poseEstimate, poseVelocity, poseAcceleration, trajectorySequence.getCurrentTrajectory().getTargetPose(), trajectorySequence.getCurrentTrajectory().getTargetMotionState());
//
                if (trajectorySequence.getCurrentTrajectory().targetEndpoint()) {//trajectorySequence.getCurrentTrajectory().targetEndpoint()
                    targetPose = trajectorySequence.getCurrentTrajectory().endPose();
                    followState = FollowState.TARGET_END_POINT;
                }

                break;
            case POINT_TURN:
                //tbd
                //might use index marker to trigger
                break;
            case TARGET_END_POINT:
                trajectorySequence.getCurrentTrajectory().updateTargetPoint(poseEstimate);
                trajectorySequence.updateGlobalTemporalMarkers();

                motorPowers = endpointController.calculatePowers(poseEstimate, poseVelocity, poseAcceleration, trajectorySequence.getCurrentTrajectory().endPose());

//                if (poseEstimate.minus(targetPose).inRange(trajectorySequence.getCurrentTrajectory().getEndError())) {
//                    delayTime = Math.max(trajectorySequence.getCurrentTrajectory().getMinTime()-trajectoryTime.seconds(), trajectorySequence.getCurrentTrajectory().getEndDelay());
//                    trajectoryTime.reset();
//                    followState = FollowState.NEXT_TRAJECTORY_DELAY;
//                }
                break;
            case NEXT_TRAJECTORY_DELAY:
//                motorPowers = pidController.calculatePowers(poseEstimate, poseVelocity, poseAcceleration, targetPose, new Pose2d(0, 0 ,0));

                if (trajectoryTime.seconds()>delayTime) {
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
                break;
        }

        lastMotorPowers = motorPowers;
        return motorPowers;
    }

    public EndpointController getEndpointController() {
        return endpointController;
    }

    public TrajectorySequence getTrajectorySequence() {
        return trajectorySequence;
    }

    public FollowState getFollowState() {
        return followState;
    }

}