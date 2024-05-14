package com.reefsharklibrary.localizers;

import static org.apache.commons.math3.util.Precision.EPSILON;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.TimePose2d;
import com.reefsharklibrary.data.Vector2d;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class TwoWheelLocalizer implements Localizer {

    public final double perpendicularX;
    public final double parallelY;

    DecompositionSolver forwardSolver;

    double[] prevWheelPositions;
    double prevHeading;

    // External interfaces
    public Pose2d relativeRobotMovement = new Pose2d(0, 0, 0);
    public LinkedList<TimePose2d> prevPositions = new LinkedList<>();
    public LinkedList<TimePose2d> prevVelocities = new LinkedList<>();

    private int maxHistorySize = 200;


    private Pose2d poseEstimate = new Pose2d(0, 0, 0);


    /*
    * Thanks to https://github.com/mrjmac/7161-powerplay/blob/main/TeamCode/src/main/java/OceanCrashPurePursuit/autonomous/odometry/TwoWheelTrackingLocalizer.java#L44 for the localizer code
    * */
    public TwoWheelLocalizer(double perpendicularX, double parallelY) {
        this.perpendicularX = perpendicularX;
        this.parallelY = parallelY;

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);

        inverseMatrix.setEntry(0, 0, 1);
        inverseMatrix.setEntry(0, 1, 0);
        inverseMatrix.setEntry(0, 2, -parallelY);

        inverseMatrix.setEntry(1, 0, 0);
        inverseMatrix.setEntry(1, 1, 1);
        inverseMatrix.setEntry(1, 2, perpendicularX);

        inverseMatrix.setEntry(2, 2, 1);

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();

        if (!forwardSolver.isNonSingular()) {
            throw new IllegalArgumentException("The specified configuration cannot support full localization");
        }

        //giving it initial values of 0
        prevWheelPositions = new double[2];

        prevPositions.add(new TimePose2d(relativeRobotMovement));
        prevVelocities.add(new TimePose2d(new Pose2d(0, 0, 0)));
    }

    @Override
    public void update() {
        update(0, 0, 0);
    }


    public void update(double parallel, double perpendicular, double heading) {
        double[] deltas = new double[] {
                parallel-prevWheelPositions[0],
                perpendicular-prevWheelPositions[1],
               heading-prevHeading
        };

        prevWheelPositions[0] = parallel;
        prevWheelPositions[1] = perpendicular;
        prevHeading = heading;

        updateFromRelative(deltas);
        updatePoseVelocitiy();

        if (prevPositions.size()>maxHistorySize) {
            prevPositions.removeFirst();
        }

        if (prevVelocities.size()>maxHistorySize) {
            prevVelocities.removeFirst();
        }
    }

    public void updateFromRelative(double[] deltas) {
        RealMatrix m = MatrixUtils.createRealMatrix(new double[][] {deltas});

        RealMatrix rawPoseDelta = forwardSolver.solve(m.transpose());

        Pose2d robotPoseDelta = new Pose2d(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        );

        relativeRobotMovement = relativeRobotMovement.plus(robotPoseDelta);
        poseEstimate = relativeOdometryUpdate(poseEstimate, robotPoseDelta);
        prevPositions.add(new TimePose2d(poseEstimate));
    }

    private static Pose2d relativeOdometryUpdate(Pose2d fieldPose, Pose2d robotPoseDelta) {
        double dtheta = robotPoseDelta.getHeading();
        double sineTerm, cosTerm;

        if (approxEquals(dtheta, 0)) {
            sineTerm = 1.0 - dtheta * dtheta / 6.0;
            cosTerm = dtheta / 2.0;
        } else {
            sineTerm = Math.sin(dtheta) / dtheta;
            cosTerm = (1 - Math.cos(dtheta)) / dtheta;
        }

        Vector2d fieldPositionDelta = new Vector2d(
                sineTerm * robotPoseDelta.getX() - cosTerm * robotPoseDelta.getY(),
                cosTerm * robotPoseDelta.getX() + sineTerm * robotPoseDelta.getY()
        );

        Pose2d fieldPoseDelta = new Pose2d(fieldPositionDelta.rotate(fieldPose.getHeading()), robotPoseDelta.getHeading());

        return fieldPose.plus(fieldPoseDelta);
    }

    private static boolean approxEquals(double d1, double d2) {
        if (Double.isInfinite(d1)) {
            // Infinity - infinity is NaN, so we need a special case
            return d1 == d2;
        } else {
            return Math.abs(d1 - d2) < EPSILON;
        }
    }

    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public List<TimePose2d> getPoseHistory() {
        return prevPositions;
    }

    @Override
    public void setPoseEstimate(Pose2d pose) {
        prevPositions.clear();
        prevVelocities.clear();
        prevVelocities.add(new TimePose2d(new Pose2d(0, 0, 0)));
        poseEstimate = pose;
    }

    private void updatePoseVelocitiy() {
        if (prevPositions.size()<2) {
            prevVelocities.add(new TimePose2d(new Pose2d(0, 0, 0)));
            return;
        }

        //looks for an index up to 4 reads ago
        int oldIndex = Math.max(0, prevPositions.size()-5);

        TimePose2d old = prevPositions.get(oldIndex);
        TimePose2d cur = prevPositions.get(prevPositions.size()-1);

        prevVelocities.add(new TimePose2d(cur.minus(old).scale((double) 1000/(cur.time-old.time)), (cur.time+old.time)/2));
    }

    @Override
    public Pose2d getPoseVelocity() {
        return prevVelocities.get(prevVelocities.size()-1);
    }

    @Override
    public Pose2d getPoseAcceleration() {
        if (prevVelocities.size()<2) {
            return new Pose2d(0, 0, 0);
        }

        //looks for an index up to 4 reads ago
        int oldIndex = Math.max(0, prevVelocities.size()-5);

        TimePose2d old = prevVelocities.get(oldIndex);
        TimePose2d cur = prevVelocities.get(prevVelocities.size()-1);

        return cur.minus(old).scale((double) 1000/(cur.time-old.time));
    }

    @Override
    public void setHistoryLimit(int maxHistorySize) {
        if (maxHistorySize<2) {
            throw new RuntimeException("History size is to small");
        }
        this.maxHistorySize = maxHistorySize+1;
    }
}
