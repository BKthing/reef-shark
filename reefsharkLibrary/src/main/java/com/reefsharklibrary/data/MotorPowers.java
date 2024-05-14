package com.reefsharklibrary.data;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;

public class MotorPowers {
    //FL, BL, BR, FR
    private final List<Double> motorPowers = Arrays.asList(0.0, 0.0, 0.0, 0.0);
    private final double nominalPower = 12;

    public List<Double> getNormalizedVoltages(double voltage) {
        List<Double> normalizedMotorPowers = new ArrayList<>();
        double normalizeVal = nominalPower/voltage;

        //if voltage is less than the nominal power the motor powers are being scaled up so we have to make sure they won't exceed 1
        if (voltage<nominalPower) {
            double maxPower = 0;

            for (double power : motorPowers) {
                maxPower = Math.max(maxPower, Math.abs(power));
            }

            double maxNormalizedPower = maxPower*normalizeVal;
            if (maxNormalizedPower>1) {
                normalizeVal /= maxNormalizedPower;
            }
        }

        for (double power: motorPowers) {
            normalizedMotorPowers.add(power*normalizeVal);
        }

        return normalizedMotorPowers;
    }

    public List<Double> getRawVoltages() {
        return motorPowers;
    }

    public void reset() {
        motorPowers.set(0, 0.0);
        motorPowers.set(1, 0.0);
        motorPowers.set(2, 0.0);
        motorPowers.set(3, 0.0);

    }

    public double getRemainingPower() {
        double maxPower = 0;
        for (double power : motorPowers) {
            maxPower = Math.max(maxPower, Math.abs(power));
        }
        return 1-maxPower;
    }

    private double getScaledPower(double power) {
        double remainingPower = getRemainingPower();
        return Math.signum(power)*Math.min(Math.abs(power), remainingPower);
    }

    public void addHeading(double heading) {
        double scaledHeading = getScaledPower(heading);

        add(0, scaledHeading);
        add(1, scaledHeading);
        add(2, -scaledHeading);
        add(3, -scaledHeading);

        checkNaN();
    }

    public void addX(double x) {
        double scaledX = getScaledPower(x);

        add(0, scaledX);
        add(1, scaledX);
        add(2, scaledX);
        add(3, scaledX);

        checkNaN();
    }

    public void addY(double y) {
        double scaledY = getScaledPower(y);

        add(0, -scaledY);
        add(1, scaledY);
        add(2, -scaledY);
        add(3, scaledY);

        checkNaN();
    }

    public void addVector(Vector2d vector) {
        double remainingPower = getRemainingPower();
        double totalPower = Math.abs(vector.getY())+Math.abs(vector.getX());
        if (totalPower>remainingPower) {
            vector = vector.scale(remainingPower/totalPower);
        }

        add(0, vector.getX() - vector.getY());
        add(1, vector.getX() + vector.getY());
        add(2, vector.getX() - vector.getY());
        add(3, vector.getX() + vector.getY());

        checkNaN();
    }

    public void orderedAddPowers(Pose2d pose) {
        addHeading(pose.getHeading());
        addVector(pose.getVector2d());
    }

    public void evenlyAddPowers(Pose2d pose) {
        double remainingPower = getRemainingPower();
        double totalPower = Math.abs(pose.getY())+Math.abs(pose.getX()+Math.abs(pose.getHeading()));
        if (totalPower>remainingPower) {
            pose = pose.scale(remainingPower/totalPower);
        }

        add(0, pose.getX() - pose.getY() + pose.getHeading());
        add(1, pose.getX() + pose.getY() + pose.getHeading());
        add(2, pose.getX() - pose.getY() - pose.getHeading());
        add(3, pose.getX() + pose.getY() - pose.getHeading());

        checkNaN();
    }

    private void add(int index, double value) {
        motorPowers.set(index, motorPowers.get(index)+value);
    }

    private void checkNaN() {
        if (!Double.isFinite(motorPowers.get(0)) || !Double.isFinite(motorPowers.get(1)) || !Double.isFinite(motorPowers.get(2)) || !Double.isFinite(motorPowers.get(3))) {
            throw new RuntimeException("Non Finite Motor Powers");
        }
    }

    public static Pose2d powersToPose(List<Double> powers) {
        if (powers.size() != 4)
            throw new RuntimeException("Invalid list of motor powers");

        return new Pose2d((powers.get(1)+powers.get(2))/2, (powers.get(3)-powers.get(2))/2, (powers.get(1)-powers.get(3))/2);
    }

    public String toString() {
        return String.format("Fl: %,3.2f Fr: %,3.2f Bl: %,3.2f Br: %,3.2f", motorPowers.get(0), motorPowers.get(3), motorPowers.get(1), motorPowers.get(2));
    }
}
