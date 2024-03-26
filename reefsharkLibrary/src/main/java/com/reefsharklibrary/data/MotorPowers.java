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

        motorPowers.add(0, scaledHeading);
        motorPowers.add(1, scaledHeading);
        motorPowers.add(2, -scaledHeading);
        motorPowers.add(3, -scaledHeading);
    }

    public void addX(double x) {
        double scaledX = getScaledPower(x);

        motorPowers.add(0, scaledX);
        motorPowers.add(1, scaledX);
        motorPowers.add(2, scaledX);
        motorPowers.add(3, scaledX);
    }

    public void addY(double y) {
        double scaledY = getScaledPower(y);

        motorPowers.add(0, -scaledY);
        motorPowers.add(1, scaledY);
        motorPowers.add(2, -scaledY);
        motorPowers.add(3, scaledY);
    }

    public void addVector(Vector2d vector) {
        double remainingPower = getRemainingPower();
        double totalPower = Math.abs(vector.getY())+Math.abs(vector.getX());
        if (totalPower>remainingPower) {
            vector = vector.scale(remainingPower/totalPower);
        }

        motorPowers.add(0, vector.getX() - vector.getY());
        motorPowers.add(1, vector.getX() + vector.getY());
        motorPowers.add(2, vector.getX() - vector.getY());
        motorPowers.add(3, vector.getX() + vector.getY());
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

        motorPowers.add(0, pose.getX() - pose.getY() + pose.getHeading());
        motorPowers.add(1, pose.getX() + pose.getY() + pose.getHeading());
        motorPowers.add(2, pose.getX() - pose.getY() - pose.getHeading());
        motorPowers.add(3, pose.getX() + pose.getY() - pose.getHeading());
    }
}
