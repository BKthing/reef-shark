package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.localizers.Localizer;
import com.reefsharklibrary.misc.ElapsedTimer;

public class TrajectorySequenceRunner {
    enum FollowState {
        FOLLOW_TRAJECTORY,
        TARGET_END_POINT,
        NEXT_TRAJECTORY_DELAY,
        NO_TRAJECTORY
    }
    FollowState followState = FollowState.NO_TRAJECTORY;

    private Pose2d targetPose;

    private TrajectorySequence trajectorySequence;
    Rotation currentRotation = new Rotation(0, 2*Math.PI);
    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;

    private final PIDController pidController;

    private final EndpointController endpointController;

    private final Localizer localizer;

    private MotorPowers lastMotorPowers = new MotorPowers();

    private ElapsedTimer runTime = new ElapsedTimer();
    private final ElapsedTimer trajectoryTime = new ElapsedTimer();
    private double delayTime = 0;




    public TrajectorySequenceRunner(PIDCoeficients lateralPID, PIDCoeficients headingPID, Localizer localizer) {
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
        pidController = new PIDController(lateralPID, headingPID);
        endpointController = new EndpointController(lateralPID, headingPID);
        this.localizer = localizer;
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        this.trajectorySequence = trajectorySequence;
        followState = FollowState.FOLLOW_TRAJECTORY;
        trajectoryTime.reset();
    }

    public MotorPowers update() {
        MotorPowers motorPowers = new MotorPowers();

        switch (followState) {
            case FOLLOW_TRAJECTORY:
                trajectorySequence.getCurrentTrajectory().updateTargetPoint(localizer.getPoseEstimate());
                motorPowers = pidController.calculatePowers(localizer.getPoseEstimate(), localizer.getPoseVelocity(), trajectorySequence.getCurrentTrajectory().getTargetPose(), trajectorySequence.getCurrentTrajectory().getTargetMotionState());

                if (trajectorySequence.getCurrentTrajectory().targetEndpoint()) {
                    targetPose = trajectorySequence.getCurrentTrajectory().endPose();
                    followState = FollowState.TARGET_END_POINT;
                }

                break;
            case TARGET_END_POINT:
                trajectorySequence.getCurrentTrajectory().updateTargetPoint(localizer.getPoseEstimate());
                motorPowers = endpointController.calculatePowers(localizer.getPoseEstimate(), localizer.getPoseVelocity(), trajectorySequence.getCurrentTrajectory().endPose());

                if (localizer.getPoseEstimate().minus(targetPose).inRange(trajectorySequence.getCurrentTrajectory().getEndError())) {
                    delayTime = Math.max(trajectorySequence.getCurrentTrajectory().getMinTime()-trajectoryTime.seconds(), trajectorySequence.getCurrentTrajectory().getEndDelay());
                    trajectoryTime.reset();
                }
                break;
            case NEXT_TRAJECTORY_DELAY:
                motorPowers = pidController.calculatePowers(localizer.getPoseEstimate(), localizer.getPoseVelocity(), targetPose, new Pose2d(0, 0 ,0));

                if (trajectoryTime.seconds()>delayTime) {
                    trajectorySequence.nextTrajectory();

                    if (trajectorySequence.isFinished()) {
                        followState = FollowState.NO_TRAJECTORY;
                    } else {
                        trajectoryTime.reset();
                        followState = FollowState.FOLLOW_TRAJECTORY;
                    }
                }
                break;
        }

        lastMotorPowers = motorPowers;
        return motorPowers;
    }

}