package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Point;
import com.reefsharklibrary.data.Pose2d;
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



    public ConstantAccelSolver() {
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


        double A = getA();
        double B = getB();
        double C = getC();
        double D = getD();
        double E = getE();
        double F = getF();
        double G = getG();
        double K = getK();

        double finalTime = getFinalTime();

        double Uf = (Math.sqrt(A) * finalTime) + (0.5 * Math.sqrt(A) * B);
        double Ui =  (0.5 * Math.sqrt(A) * B);

        double ABCf = A * Math.pow(finalTime, 2) + B * finalTime + C;

        double sinABCf = Math.sin(ABCf);
        double cosABCf = Math.cos(ABCf);
        double sinC = Math.sin(C);
        double cosC = Math.cos(C);

        double D2A = D / (2 * A);
        double F2A = F / (2 * A);

        double EBD = (E - (B * D2A)) / Math.sqrt(A);
        double GBF = (G - (B * F2A)) / Math.sqrt(A);

        double cosK = Math.cos(K);
        double sinK = Math.sin(K);

        double fresnelCos = FresnelEstimator.C(Uf) - FresnelEstimator.C(Ui);
        double fresnelSin = FresnelEstimator.S(Uf) - FresnelEstimator.S(Ui);

        double fieldDeltaX = (D2A * sinABCf) - (D2A * sinC) + (EBD * (-sinK * fresnelSin + cosK * fresnelCos))
                            - ((-F2A * cosABCf)) + (F2A * cosC) + (GBF * (sinK * fresnelCos + cosK * fresnelSin));

        double fieldDeltaY = ((-D2A * cosABCf) + (D2A * cosC) + (EBD * (sinK * fresnelCos + cosK * fresnelSin)))
                            - ((F2A * sinABCf) - (F2A * sinC) + (GBF * (-sinK * fresnelSin + cosK * fresnelCos)));

        prevTime += finalTime;

        return new Vector2d(fieldDeltaX, fieldDeltaY);
    }

    private double getA () {
        return  (
                deltaHList.get(0).getTime() / ( (deltaHList.get(0).getVal() - deltaHList.get(1).getVal()) * (deltaHList.get(0).getVal() - deltaHList.get(2).getVal()))
                + deltaHList.get(1).getTime() / ( (deltaHList.get(1).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(1).getVal() - deltaHList.get(2).getVal()))
                + deltaHList.get(2).getTime() / ( (deltaHList.get(2).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(2).getVal() - deltaHList.get(1).getVal()))
        );
    }

    private double getB() {
        return (
                (deltaHList.get(0).getTime() * 2) / ( (deltaHList.get(0).getVal() - deltaHList.get(1).getVal()) * (deltaHList.get(0).getVal() - deltaHList.get(2).getVal()))
                + (deltaHList.get(1).getTime() * 2) / ( (deltaHList.get(1).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(1).getVal() - deltaHList.get(2).getVal()))
                + (deltaHList.get(2).getTime() * 2) / ( (deltaHList.get(2).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(2).getVal() - deltaHList.get(1).getVal()))
                - (deltaHList.get(0).getTime() * (deltaHList.get(1).getVal() + deltaHList.get(2).getVal())) / ( (deltaHList.get(0).getVal() - deltaHList.get(1).getVal()) * (deltaHList.get(0).getVal() - deltaHList.get(2).getVal()))
                - (deltaHList.get(1).getTime() * (deltaHList.get(0).getVal() + deltaHList.get(2).getVal())) / ( (deltaHList.get(1).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(1).getVal() - deltaHList.get(2).getVal()))
                - (deltaHList.get(2).getTime() * (deltaHList.get(0).getVal() + deltaHList.get(1).getVal())) / ( (deltaHList.get(2).getVal() - deltaHList.get(0).getVal()) * (deltaHList.get(2).getVal() - deltaHList.get(1).getVal()))
        );
    }

    private double getC() {
        double h1 = deltaHList.get(0).getVal();
        double h2 = deltaHList.get(1).getVal();
        double h3 = deltaHList.get(2).getVal();
        double th1 = deltaHList.get(0).getTime();
        double th2 = deltaHList.get(1).getTime();
        double th3 = deltaHList.get(2).getTime();

        return ( ((h2 * h3 * th1) / ((h1 - h2) * (h1 - h3)))
                + ((h1 * h3 * th2) / ((h2 - h1) * (h2 - h3)))
                + ((h1 * h2 * th3) / ((h3 - h1) * (h3 - h2)))
                - (((h2 + h3) * th1) / ((h1 - h2) * (h1 - h3)))
                - (((h1 + h3) * th2) / ((h2 - h1) * (h2 - h3)))
                - (((h1 + h2) * th3) / ((h3 - h1) * (h3 - h2)))
        );
    }

    private double getD() {
        return (
                (2 * deltaXList.get(0).getTime()) / ( (deltaXList.get(0).getVal() - deltaXList.get(1).getVal()) * (deltaXList.get(0).getVal() - deltaXList.get(2).getVal()))
                + (2 * deltaXList.get(1).getTime()) / ( (deltaXList.get(1).getVal() - deltaXList.get(0).getVal()) * (deltaXList.get(1).getVal() - deltaXList.get(2).getVal()))
                + (2 * deltaXList.get(2).getTime()) / ( (deltaXList.get(2).getVal() - deltaXList.get(0).getVal()) * (deltaXList.get(2).getVal() - deltaXList.get(1).getVal()))
        );
    }

    private double getE() {
        double x1 = deltaXList.get(0).getVal();
        double x2 = deltaXList.get(1).getVal();
        double x3 = deltaXList.get(2).getVal();
        double tx1 = deltaXList.get(0).getTime();
        double tx2 = deltaXList.get(1).getTime();
        double tx3 = deltaXList.get(2).getTime();

        return ( ((-1 * (x2 + x3) * tx1) / ((x1 - x2) * (x1 - x3)))
                - (( (x1 + x3) * tx2) / ((x2 - x1) * (x2 - x3)))
                - (((x1 + x2) * tx3) / ((x3 - x1) * (x3 - x2)))
        );
    }

    private double getF() {
        return (
                (2 * deltaYList.get(0).getTime()) / ( (deltaYList.get(0).getVal() - deltaYList.get(1).getVal()) * (deltaYList.get(0).getVal() - deltaYList.get(2).getVal()))
                + (2 * deltaYList.get(1).getTime()) / ( (deltaYList.get(1).getVal() - deltaYList.get(0).getVal()) * (deltaYList.get(1).getVal() - deltaYList.get(2).getVal()))
                + (2 * deltaYList.get(2).getTime()) / ( (deltaYList.get(2).getVal() - deltaYList.get(0).getVal()) * (deltaYList.get(2).getVal() - deltaYList.get(1).getVal()))
        );
    }

    private double getG() {
        double y1 = deltaYList.get(0).getVal();
        double y2 = deltaYList.get(1).getVal();
        double y3 = deltaYList.get(2).getVal();
        double ty1 = deltaYList.get(0).getTime();
        double ty2 = deltaYList.get(1).getTime();
        double ty3 = deltaYList.get(2).getTime();

        return ( ((-1 * (y2 + y3) * ty1) / ((y1 - y2) * (y1 - y3)))
                + ((-1 * (y1 + y3) * ty2) / ((y2 - y1) * (y2 - y3)))
                + ((-1 * (y1 + y2) * ty3) / ((y3 - y1) * (y3 - y2)))
        );
    }

    private double getK() {
        return (getC() - Math.pow( ( 1 / (2 * Math.sqrt(getA())) ) * getB(), 2));
    }

    private double getFinalTime() {
        return Math.min(deltaXList.get(2).getTime(), Math.min(deltaYList.get(2).getTime(), deltaHList.get(2).getTime()));
    }

    @Override
    public double getPreviousTime() {
        return prevTime;
    }

    @Override
    public void setPoseEstimate(Pose2d pose) {
        deltaXList.clear();
        deltaYList.clear();
        deltaHList.clear();
        prevTime = 0;

        deltaXList.add(new Point(pose.getX(), 0));
        deltaXList.add(new Point(pose.getX(), 0));
        deltaXList.add(new Point(pose.getX(), 0));

        deltaYList.add(new Point(pose.getY(), 0));
        deltaYList.add(new Point(pose.getY(), 0));
        deltaYList.add(new Point(pose.getY(), 0));

        deltaHList.add(new Point(pose.getHeading(), 0));
        deltaHList.add(new Point(pose.getHeading(), 0));
        deltaHList.add(new Point(pose.getHeading(), 0));


    }


}
