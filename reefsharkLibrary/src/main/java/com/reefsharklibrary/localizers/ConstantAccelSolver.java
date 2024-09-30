package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Point;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.TimePose2d;
import com.reefsharklibrary.data.Vector2d;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class ConstantAccelSolver implements Solver {

    LinkedList<Point> deltaXList = new LinkedList<>();
    LinkedList<Point> deltaYList = new LinkedList<>();
    LinkedList<Point> deltaHList = new LinkedList<>();

    double prevTime = 0;

    double A, B, C, D, E, F, G;



    public ConstantAccelSolver() {
//        deltaXList.add(new Point(-6, .5));
//        deltaXList.add(new Point(-3, 1.4));
//        deltaXList.add(new Point(5, 3));
//
//        deltaYList.add(new Point(1, 1));
//        deltaYList.add(new Point(-5, 2));
//        deltaYList.add(new Point(3, 4));
//
//        deltaHList.add(new Point(-1.96, 0.12));
//        deltaHList.add(new Point(1.55, 0.21));
//        deltaHList.add(new Point(0.12, 0.84));

        deltaXList.add(new Point(0, 0));
        deltaXList.add(new Point(0, 0));
        deltaXList.add(new Point(0, 0));

        deltaYList.add(new Point(0, 0));
        deltaYList.add(new Point(0, 0));
        deltaYList.add(new Point(0, 0));

        deltaHList.add(new Point(0, 0));
        deltaHList.add(new Point(0, 0));
        deltaHList.add(new Point(0, 0));
    }


    @Override
    public Vector2d getRelativeFieldMovement(Point deltaX, Point deltaY, Point deltaHeading) {
        if (!Double.isFinite(deltaX.getVal()) || !Double.isFinite(deltaY.getVal()) || !Double.isFinite(deltaHeading.getVal())) {
            throw new RuntimeException("non finite raw deltas: " + deltaX.getVal() + ", " + deltaY.getVal() );
        }


        deltaXList.removeFirst();
        deltaYList.removeFirst();
        deltaHList.removeFirst();

        deltaYList.add(deltaY);
        deltaXList.add(deltaX);
        deltaHList.add(deltaHeading);

        for (int i = 0; i <3; i ++) {
            deltaYList.get(i).setTimeOffset(prevTime);
            deltaXList.get(i).setTimeOffset(prevTime);
            deltaHList.get(i).setTimeOffset(prevTime);
        }


        A = getA();
        int direction = 1;
        if (A < 0) {
            direction = -1;
            A = -A;
        }
        if (!Double.isFinite(A)) {
            throw new RuntimeException("A");
        }
        B = direction * getB();

        if (!Double.isFinite(B)) {
            throw new RuntimeException("B");
        }
        C = direction * getC();
        if (!Double.isFinite(C)) {
            throw new RuntimeException("C");
        }
        D = direction * getD();
        if (!Double.isFinite(D)) {
            throw new RuntimeException("D");
        }
        E = direction * getE();
        if (!Double.isFinite(E)) {
            throw new RuntimeException("E");
        }
        F = getF();
        if (!Double.isFinite(F)) {
            throw new RuntimeException("F");
        }
        G = getG();
        if (!Double.isFinite(G)) {
            throw new RuntimeException("G");
        }
        double K = getK();
        if (!Double.isFinite(K)) {
            throw new RuntimeException("K");
        }

        double changeInTime = getFinalTime();

        double Uf = Math.sqrt(A) * changeInTime + divideTerms(B, 2 * Math.sqrt(A));
        double Ui =  (divideTerms(B, 2 * Math.sqrt(A)));

        double ABCf = A * Math.pow(changeInTime, 2) + B * changeInTime + C;

        double sinABCf = Math.sin(ABCf);
        double cosABCf = Math.cos(ABCf);
        double sinC = Math.sin(C);
        double cosC = Math.cos(C);

        double D2A = divideTerms(D, (2 * A));
        double F2A = divideTerms(F,  (2 * A));

        double EBD = divideTerms(E - (B * D2A), Math.sqrt(A));
        double GBF = divideTerms(G - (B * F2A), Math.sqrt(A));

        double cosK = Math.cos(K);
        double sinK = Math.sin(K);

        double fresnelCos = FresnelEstimator.C(Math.sqrt(2/Math.PI) * Uf) - FresnelEstimator.C(Math.sqrt(2/Math.PI) * Ui);
        double fresnelSin = FresnelEstimator.S(Math.sqrt(2/Math.PI) * Uf)  - FresnelEstimator.S(Math.sqrt(2/Math.PI) * Ui);


        double fieldDeltaX = direction * ((D2A * sinABCf) - (D2A * sinC) + (EBD * ((-sinK * fresnelSin) + cosK * fresnelCos))
                            - ((-F2A * cosABCf) + (F2A * cosC) + (GBF * (sinK * fresnelCos + cosK * fresnelSin))));


        double fieldDeltaY = ((-D2A * cosABCf) + (D2A * cosC) + (EBD * (sinK * fresnelCos + cosK * fresnelSin)))
                            - ((F2A * sinABCf) - (F2A * sinC) + (GBF * (-sinK * fresnelSin + cosK * fresnelCos)));

        prevTime += changeInTime;


        if (!Double.isFinite(fieldDeltaX)|| !Double.isFinite(fieldDeltaY)) {
            throw new RuntimeException("non finite pose deltas: " + fieldDeltaX + ", " + fieldDeltaY );
        }

        return new Vector2d(fieldDeltaX, fieldDeltaY);
    }

    private double getA () {
        return  (
                divideTerms (deltaHList.get(0).getTime(),  ( (deltaHList.get(0).getVal() - deltaHList.get(1).getVal()) * (deltaHList.get(0).getVal() - deltaHList.get(2).getVal())))
                + divideTerms(deltaHList.get(1).getTime(), ( (deltaHList.get(1).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(1).getVal() - deltaHList.get(2).getVal())))
                + divideTerms(deltaHList.get(2).getTime(), ( (deltaHList.get(2).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(2).getVal() - deltaHList.get(1).getVal())))
        );
    }

    private double getB() {
        return (
                divideTerms(deltaHList.get(0).getTime() * 2,  (deltaHList.get(0).getVal() - deltaHList.get(1).getVal()) * (deltaHList.get(0).getVal() - deltaHList.get(2).getVal()))
                + divideTerms(deltaHList.get(1).getTime() * 2, (deltaHList.get(1).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(1).getVal() - deltaHList.get(2).getVal()))
                + divideTerms(deltaHList.get(2).getTime() * 2,  (deltaHList.get(2).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(2).getVal() - deltaHList.get(1).getVal()))
                - divideTerms(deltaHList.get(0).getTime() * (deltaHList.get(1).getVal() + deltaHList.get(2).getVal()),  (deltaHList.get(0).getVal() - deltaHList.get(1).getVal()) * (deltaHList.get(0).getVal() - deltaHList.get(2).getVal()))
                - divideTerms(deltaHList.get(1).getTime() * (deltaHList.get(0).getVal() + deltaHList.get(2).getVal()),  (deltaHList.get(1).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(1).getVal() - deltaHList.get(2).getVal()))
                - divideTerms(deltaHList.get(2).getTime() * (deltaHList.get(0).getVal() + deltaHList.get(1).getVal()),  (deltaHList.get(2).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(2).getVal() - deltaHList.get(1).getVal()))
        );
    }

    private double getC() {


        return (
                divideTerms(deltaHList.get(1).getVal() * deltaHList.get(2).getVal() * deltaHList.get(0).getTime(), (deltaHList.get(0).getVal() - deltaHList.get(1).getVal()) * (deltaHList.get(0).getVal() - deltaHList.get(2).getVal()))
                + divideTerms(deltaHList.get(0).getVal() * deltaHList.get(2).getVal() * deltaHList.get(1).getTime(), (deltaHList.get(1).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(1).getVal() - deltaHList.get(2).getVal()))
                + divideTerms(deltaHList.get(0).getVal() * deltaHList.get(1).getVal() * deltaHList.get(2).getTime(), (deltaHList.get(2).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(2).getVal() - deltaHList.get(1).getVal()))
                + divideTerms((deltaHList.get(1).getVal() + deltaHList.get(2).getVal()) * deltaHList.get(0).getTime() * -1, (deltaHList.get(0).getVal() - deltaHList.get(1).getVal()) * (deltaHList.get(0).getVal() - deltaHList.get(2).getVal()))
                + divideTerms((deltaHList.get(0).getVal() + deltaHList.get(2).getVal()) * deltaHList.get(1).getTime() * -1, (deltaHList.get(1).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(1).getVal() - deltaHList.get(2).getVal()))
                + divideTerms((deltaHList.get(0).getVal() + deltaHList.get(1).getVal()) * deltaHList.get(2).getTime() * -1,(deltaHList.get(2).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(2).getVal() - deltaHList.get(1).getVal()))
        );
    }

    private double getD() {
        return (
                (divideTerms(2 * deltaXList.get(0).getTime(),  (deltaXList.get(0).getVal() - deltaXList.get(1).getVal()) * (deltaXList.get(0).getVal() - deltaXList.get(2).getVal()))
                + divideTerms(2 * deltaXList.get(1).getTime(),  (deltaXList.get(1).getVal() - deltaXList.get(0).getVal()) * (deltaXList.get(1).getVal() - deltaXList.get(2).getVal()))
                + divideTerms(2 * deltaXList.get(2).getTime(), (deltaXList.get(2).getVal() - deltaXList.get(0).getVal()) * (deltaXList.get(2).getVal() - deltaXList.get(1).getVal())))
        );
    }

    private double getE() {
        double x1 = deltaXList.get(0).getVal();
        double x2 = deltaXList.get(1).getVal();
        double x3 = deltaXList.get(2).getVal();
        double tx1 = deltaXList.get(0).getTime();
        double tx2 = deltaXList.get(1).getTime();
        double tx3 = deltaXList.get(2).getTime();

        return (
                divideTerms(-1 * (x2 + x3) * tx1,  (x1 - x2) * (x1 - x3))
                - divideTerms( (x1 + x3) * tx2, (x2 - x1) * (x2 - x3))
                - divideTerms((x1 + x2) * tx3,  (x3 - x1) * (x3 - x2))
        );
    }

    private double getF() {
        return (
                divideTerms(2 * deltaYList.get(0).getTime(),  (deltaYList.get(0).getVal() - deltaYList.get(1).getVal()) * (deltaYList.get(0).getVal() - deltaYList.get(2).getVal()))
                + divideTerms(2 * deltaYList.get(1).getTime(),  (deltaYList.get(1).getVal() - deltaYList.get(0).getVal()) * (deltaYList.get(1).getVal() - deltaYList.get(2).getVal()))
                + divideTerms(2 * deltaYList.get(2).getTime(), (deltaYList.get(2).getVal() - deltaYList.get(0).getVal()) * (deltaYList.get(2).getVal() - deltaYList.get(1).getVal()))
        );
    }

    private double getG() {
        double y1 = deltaYList.get(0).getVal();
        double y2 = deltaYList.get(1).getVal();
        double y3 = deltaYList.get(2).getVal();
        double ty1 = deltaYList.get(0).getTime();
        double ty2 = deltaYList.get(1).getTime();
        double ty3 = deltaYList.get(2).getTime();

        return (
                divideTerms(-1 * (y2 + y3) * ty1, (y1 - y2) * (y1 - y3))
                + divideTerms(-1 * (y1 + y3) * ty2, (y2 - y1) * (y2 - y3))
                + divideTerms(-1 * (y1 + y2) * ty3, (y3 - y1) * (y3 - y2))
        );
    }

    private double getK() {
        if (!Double.isFinite(Math.sqrt(A)))
            throw new RuntimeException("K during" + Math.sqrt(A));
        return (C - Math.pow(divideTerms(1, 2 * Math.sqrt(A)) * B, 2));
//        return (C - Math.pow(divideTerms(1,  2 * Math.sqrt(A)) * B, 2));
    }

    private double getFinalTime() {
        return Math.min(deltaXList.get(2).getTime(), Math.min(deltaYList.get(2).getTime(), deltaHList.get(2).getTime()));
    }

    private double divideTerms(double num, double den) {
        if (den == 0)
            return 0;
        return num/den;
    }

    @Override
    public double getPreviousTime() {
        return prevTime;
    }

    @Override
    public void setPoseEstimate(Pose2d pose) {
        setPoseEstimate(new TimePose2d(pose, prevTime));
    }

    //sets pose estimate while reseting robot velocities and accelerations
    @Override
    public void setPoseEstimate(TimePose2d pose) {
        deltaXList.clear();
        deltaYList.clear();
        deltaHList.clear();

        prevTime = pose.getTime();

        deltaXList.add(new Point(pose.getX(), prevTime));
        deltaXList.add(new Point(pose.getX(), prevTime));
        deltaXList.add(new Point(pose.getX(), prevTime));

        deltaYList.add(new Point(pose.getY(), prevTime));
        deltaYList.add(new Point(pose.getY(), prevTime));
        deltaYList.add(new Point(pose.getY(), prevTime));

        deltaHList.add(new Point(pose.getHeading(), prevTime));
        deltaHList.add(new Point(pose.getHeading(), prevTime));
        deltaHList.add(new Point(pose.getHeading(), prevTime));


    }

    @Override
    public void updatePoseEstimate(Pose2d pose) {
        updatePoseEstimate(new TimePose2d(pose, prevTime));
    }

    //sets pose estimate while preserving robot velocities and accelerations
    @Override
    public void updatePoseEstimate(TimePose2d pose) {
        Pose2d deltaPose = new Pose2d(pose.getX()-deltaXList.get(2).getVal(), pose.getY()-deltaYList.get(2).getVal(), pose.getHeading()-deltaHList.get(2).getVal());

        deltaXList.set(0, new Point(deltaXList.get(1).getVal()+deltaPose.getX(), deltaXList.get(1).getTime()));
        deltaXList.set(1, new Point(deltaXList.get(2).getVal()+deltaPose.getX(), deltaXList.get(2).getTime()));
        deltaXList.set(2, new Point(pose.getX(), pose.getTime()));

        deltaYList.set(0, new Point(deltaYList.get(1).getVal()+deltaPose.getY(), deltaYList.get(1).getTime()));
        deltaYList.set(1, new Point(deltaYList.get(2).getVal()+deltaPose.getY(), deltaYList.get(2).getTime()));
        deltaYList.set(2, new Point(pose.getY(), pose.getTime()));

        deltaHList.set(0, new Point(deltaHList.get(1).getVal()+deltaPose.getHeading(), deltaHList.get(1).getTime()));
        deltaHList.set(1, new Point(deltaHList.get(2).getVal()+deltaPose.getHeading(), deltaHList.get(2).getTime()));
        deltaHList.set(2, new Point(pose.getHeading(), pose.getTime()));

    }
}
