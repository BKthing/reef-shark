package com.reefsharklibrary.geometries;

import com.reefsharklibrary.data.Vector2d;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class CubicBezierCurve implements Geometry {
    private final List<BezierDataPoint> points;
    private double prevSearchDistance= -1;
    private int prevSearchIndex = 0;
    private final Vector2d startPoint;
    private final Vector2d endPoint;

    private final Function<Double, Double> x;
    private final Function<Double, Double> y;
    private final Function<Double, Double> dx;
    private final Function<Double, Double> dy;


    public CubicBezierCurve(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4) {
        startPoint = p1;
        endPoint = p4;

        x = (Double t) -> {
            final double oneMinusT = 1-t;
            return (((oneMinusT*p1.getX())+3*p2.getX()*t)*oneMinusT+3*Math.pow(t, 2)*p3.getX())*oneMinusT+Math.pow(t, 3)*p4.getX();
//            return Math.pow(1-t, 3)*p1.getX() + 3*Math.pow(1-t, 2)*t*p2.getX() + 3*(t-1)*Math.pow(t, 2)*p3.getX() + Math.pow(t, 3)*p4.getX();
        };
        y = (Double t) -> {
            final double oneMinusT = 1-t;
            return (((oneMinusT*p1.getY())+3*p2.getY()*t)*oneMinusT+3*Math.pow(t, 2)*p3.getY())*oneMinusT+Math.pow(t, 3)*p4.getY();
//            return Math.pow(1-t, 3)*p1.getY() + 3*Math.pow(1-t, 2)*t*p2.getY() + 3*(t-1)*Math.pow(t, 2)*p3.getY() + Math.pow(t, 3)*p4.getY();
        };

        dx = (Double t) -> - 3*Math.pow(t-1, 2)*p1.getX() - 3*(1-t)*(3*t-1)*p2.getX() + 3*(2-3*t)*t*p3.getX() + 3*Math.pow(t, 2)* p4.getX();
        dy = (Double t) -> - 3*Math.pow(t-1, 2)*p1.getY() - 3*(1-t)*(3*t-1)*p2.getY() + 3*(2-3*t)*t*p3.getY() + 3*Math.pow(t, 2)* p4.getY();

        points = generate(.01);
    }

    @Override
    public Vector2d getPoint(double distance) {
        return points.get(findClosestDataPoint(distance)).getVector2d();
    }

    @Override
    public double tangentAngle(double distance) {
        //set t = to the t of a point with the closest distance
        double t = points.get(findClosestDataPoint(distance)).getT();
        return Math.atan2(dy.apply(t), dx.apply(t));
    }

    @Override
    public double getTotalDistance() {
        return points.get(points.size()-1).getDistance();
    }

    @Override
    public Vector2d startPoint() {
        return points.get(0).getVector2d();
    }

    @Override
    public Vector2d endPoint() {
        return points.get(points.size()-1).getVector2d();
    }



    private int findClosestDataPoint(double distance) {
        if (distance == prevSearchDistance) {
            return prevSearchIndex;
        } else {
            int n = points.size();

            //checking edge cases
            if (distance<=0) {
                return 0;
            }
            if (distance>=points.get(points.size()-1).getDistance()) {
                return points.size()-1;
            }

            //binary search
            int i = 0, j = n, mid = 0;
            while (i<j) {
                mid = (i+j)/2;

                if (points.get(mid).getDistance() == distance) {
                    return mid;
                }
                //if less than search the left side
                if (distance<points.get(mid).getDistance()) {
                    if (mid>0 && distance>points.get(mid-1).getDistance()) {
                        return getClosest(mid-1, mid, distance);
                    }

                    j = mid;
                } else {
                    if (mid < n-1 && distance < points.get(mid+1).getDistance()) {
                        return getClosest(mid, mid+1, distance);
                    }
                    i = mid+1;
                }
            }
            return mid;
        }

    }

    private int getClosest(int index1, int index2, double distance) {
        if (distance-points.get(index1).getDistance() >= points.get(index2).getDistance()-distance)
            return index2;
        else
            return index1;
    }

    private List<BezierDataPoint> generate(double resolution) {
        List<BezierDataPoint> points = new ArrayList<>();

        //the headings will be
        points.add(new BezierDataPoint(startPoint));
        for (double i = resolution; i<1-resolution; i+=resolution) {
            points.add(new BezierDataPoint(new Vector2d(x.apply(i), y.apply(i)), points.get(points.size()-1), i));
        }

        points.add(new BezierDataPoint(endPoint, points.get(points.size()-1) ,1));

        return points;
    }
}
