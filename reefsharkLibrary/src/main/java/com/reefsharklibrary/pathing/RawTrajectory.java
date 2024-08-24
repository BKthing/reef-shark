package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.pathing.data.IndexCallMarker;
import com.reefsharklibrary.pathing.data.TemporalCallMarker;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RawTrajectory implements RawTrajectoryInterface {
    private final List<Pose2d> positions = new ArrayList<>();

    private List<IndexCallMarker> callMarkers = new ArrayList<>();
    private List<TemporalCallMarker> temporalCallMarkers = new ArrayList<>();

    private Pose2d followError = new Pose2d(1, 1 ,3);
    private Pose2d endError = new Pose2d(.5, .5 ,2);

    private double endDelay = 0;
    private double minTime = 0;

    private double targetEndDistance = 6;

    private final double initialDistance;
    private double pathDistance = 0;



    RawTrajectory(double initialDistance) {
        this.initialDistance = initialDistance;

    }

    @Override
    public double getAccumulatedDistance() {
        return pathDistance;
    }

    @Override
    public double getTotalDistance() {
        return initialDistance+pathDistance;
    }

    @Override
    public Pose2d getLastPose() {
        return positions.get(positions.size()-1);
    }

    @Override
    public List<Pose2d> getPoseList() {
        return positions;
    }

    public void addPose(Pose2d position, double distance) {
        this.positions.add(position);
        pathDistance += distance;
    };

    //total distance is the distance the points are adding to the trajectory
    public void addSet(List<Pose2d> points, double totalDistance) {
        this.positions.addAll(points);
        this.pathDistance += totalDistance;
    }

    public void addTangentSet(List<Pose2d> points, double totalDistance) {
        //adds all but the first term to the list
        for (int i = 1; i<points.size(); i++) {
            positions.add(points.get(i));
        }

        this.pathDistance += totalDistance;
    }

    public void addCallMarker(IndexCallMarker callMarker) {
        callMarkers.add(callMarker);
    }

    public void setCallMarkers(List<IndexCallMarker> callMarkers) {
        this.callMarkers = callMarkers;
    }

    public void addTemporalCallMarker(TemporalCallMarker temporalCallMarker) {
        temporalCallMarkers.add(temporalCallMarker);
    }

    public void setTemporalCallMarkers(List<TemporalCallMarker> temporalCallMarkers) {
        this.temporalCallMarkers = temporalCallMarkers;
    }

    public void setFollowError(Pose2d followError) {
        this.followError = followError;
    }

    public void setEndError(Pose2d endError) {
        this.endError = endError;
    }

    public void setEndDelay(double endDelay) {
        this.endDelay = endDelay;
    }

    public void setTargetEndDistance(double targetEndDistance) {
        this.targetEndDistance = targetEndDistance;
    }

    public TrajectoryInterface build(ConstraintSet constraints, double resolution) {
        if (temporalCallMarkers.size()>0) {
            sortTemporalMarkers();
            minTime = Math.max(temporalCallMarkers.get(temporalCallMarkers.size()-1).getCallTime(), minTime);
        }

        if (callMarkers.size()>0) {
            sortCallMarkers();
            indexCallMarkers(resolution);
        }

        MecanumVelocityFinder mecanumVelocityFinder = new MecanumVelocityFinder(positions, constraints);

                //calculateVelocities(constraints, resolution)
        return new Trajectory(positions, mecanumVelocityFinder.getVelocities(), callMarkers, temporalCallMarkers, followError, endError, endDelay, minTime, positions.size()- 1 - (int) (targetEndDistance/resolution));
    }

    private void sortTemporalMarkers() {
        Collections.sort(temporalCallMarkers);
    }

    private void sortCallMarkers() {
        Collections.sort(callMarkers);
    }

    private void indexCallMarkers(double resolution) {
        for(IndexCallMarker callMarker: callMarkers) {
            callMarker.setCallPosition(
                    //Assigns closest list position to markers
                    Math.min((int) Math.round(callMarker.getCallDistance()/resolution), positions.size()-1)
            );
        }
    }


    //TODO: fix this
//    private List<Pose2d> calculateVelocities(ConstraintSet constraints, double resolution) {
//        List<Pose2d> velocities = new ArrayList<>();
//
//        Pose2d prevVelocities = new Pose2d(0, 0, 0);
//
//        for (int i = 0; i<positions.size()-1; i++){
//            prevVelocities = findVelocities(positions.get(i+1).minus(positions.get(i)), prevVelocities, positions.get(i+1).getHeading(), constraints);
//            velocities.add(prevVelocities);
//        }
//
//        velocities.add(new Pose2d(0, 0, 0));
//        prevVelocities = new Pose2d(0, 0, 0);
//
//        int i = velocities.size()-1;
//
//        boolean decelerating = true;
//
//        while (decelerating) {
//            prevVelocities = findDecelVelocities(positions.get(i).minus(positions.get(i-1)), prevVelocities, positions.get(i).getHeading(), constraints);
//
//            if (poseLessThanOrEqual(velocities.get(i-1), prevVelocities) || i<2) {
//                decelerating = false;
//            } else {
//                velocities.set(i-1, prevVelocities);
//                i--;
//            }
//        }
//
//        return velocities;
//    }
//
//    private boolean poseLessThanOrEqual(Pose2d pose1, Pose2d pose2) {
//        return Math.abs(pose1.getX())<=Math.abs(pose2.getX()) && Math.abs(pose1.getY())<=Math.abs(pose2.getY());// && pose1.getHeading()<pose2.getHeading()
//    }
//
//    //this is currently a bad estimate at velocity that doesn't take into account a lot of important factors
//    //TODO: take into consideration how much force it takes to change the direction of velocity so that tight turns will work
//    private Pose2d findVelocities(Pose2d difference, Pose2d prevVelocities, double heading, ConstraintSet constraints) {
//
//        double direction = difference.getDirection();
//        Vector2d initialVelocities = prevVelocities.getVector2d().rotate(-direction);
//        double initialVelocity = initialVelocities.getX();
//        double distance = difference.getVector2d().getMagnitude();
//
//        //how much linear velocity the wheels have to exert to turn
//        double headingLinearVel = Math.abs(constraints.getWheelBaseRadius()*difference.getHeading());
//        double averageHeading = heading-difference.getHeading()/2;
//
//        double relRotation = Rotation.inRange(direction-averageHeading, 2*Math.PI, 0);
//
//        //finds how much power the mecanum drive can exert in the given direction relative to its current heading
//        double maxVel = ConstraintSet.getAdjustedMecanumVector(constraints.getMaxLinearVel(), relRotation).getMagnitude();//-headingLinearVel;
//        double maxAccel = ConstraintSet.getAdjustedMecanumVector(constraints.getMaxLinearAccel(), relRotation).getMagnitude();//-headingLinearVel;
//
//        double achievableVelocity = Math.sqrt(Math.pow(initialVelocity, 2) + 2*maxAccel*distance);
//
//        double time;
//
//        if (achievableVelocity>maxVel) {
//            time = distance/maxVel;
//        } else {
//            time = (achievableVelocity-initialVelocity)/maxAccel;
//        }
//
//        double headingVel = difference.getHeading()/time;
//
//        double maxHeadingVel = Math.min(
//                Math.sqrt(Math.pow(prevVelocities.getHeading(), 2)+2*constraints.getMaxAngularAccel()*difference.getHeading()),
//                constraints.getMaxAngularVel()
//            );
//
//        if (headingVel>maxHeadingVel) {
//            time = difference.getHeading()/maxHeadingVel;
//        }
//
//        if (time == 0) throw new RuntimeException("Time of 0");
//
//
//
//        //divides the change in position by the change in time to give the velocity
//        Pose2d velocity = difference.scale(1/time);
//        velocity.enforceFinite();
//
//        return velocity;
//    }
//
//    private Pose2d findDecelVelocities(Pose2d difference, Pose2d prevVelocities, double heading, ConstraintSet constraints) {
//        difference.enforceFinite();
//        double direction = difference.getDirection();
//        Vector2d initialVelocities = prevVelocities.getVector2d().rotate(-direction);
//        double initialVelocity = initialVelocities.getX();
//        double distance = difference.getVector2d().getMagnitude();
//
//        //how much linear velocity the wheels have to exert to turn
//        double headingLinearVel = Math.abs(constraints.getWheelBaseRadius()*difference.getHeading());
//        double averageHeading = heading-difference.getHeading()/2;
//
//        //finds how much power the mecanum drive can exert in the given direction relative to its current heading
//        double relRotation = Rotation.inRange(direction-averageHeading, 2*Math.PI, 0);
//        double maxVel = ConstraintSet.getAdjustedMecanumVector(constraints.getMaxLinearVel(), relRotation).getMagnitude();//-headingLinearVel;
//        double maxAccel = ConstraintSet.getAdjustedMecanumVector(constraints.getMaxLinearDecel(), relRotation).getMagnitude();//-headingLinearVel;
//
//        if (maxAccel == 0) throw new RuntimeException("maxAccel 0");
//
//        double achievableVelocity = Math.sqrt(Math.pow(initialVelocity, 2) + 2*maxAccel*distance);
//
//        if (!Double.isFinite(achievableVelocity)) throw new RuntimeException("Non finite achievable vel " + initialVelocity + " " + maxAccel + " " + distance);
//
//        double time;
//
//        if (achievableVelocity>maxVel) {
//            //TODO: find a better time estimate for when maxVel is greater than achievableVel
//            time = distance/maxVel;
//            if (!Double.isFinite(time)) throw new RuntimeException("Non finite time " + time);
//
//        } else {
//            time = (achievableVelocity-initialVelocity)/maxAccel;
//            if (!Double.isFinite(time)) throw new RuntimeException("Non finite time " + time);
//        }
//
//        double headingVel = difference.getHeading()/time;
//
//        double maxHeadingVel = Math.min(
//                Math.sqrt(Math.pow(prevVelocities.getHeading(), 2)+2*constraints.getMaxAngularDecel()*difference.getHeading()),
//                constraints.getMaxAngularVel()
//        );
//
//        if (headingVel>maxHeadingVel) {
//            time = difference.getHeading()/maxHeadingVel;
//        }
//
//        //divides the change in position by the change in time to give the velocity
//        return difference.scale(1/time);
//    }
}

/*
* Over complicated velocity code
*
* Vector2d adjustedMaxVel = constraints.getMaxLinearVel().minus(headingLinearVel);
        Vector2d maxVelByAccel = new Vector2d()

        double bestX;

        if (adjustedMaxVel.getX()<maxVelByAccel.getX() && adjustedMaxVel.getY()<maxVelByAccel.getY()) {
            bestX = findIdealVelocity(
                    new Vector2d(adjustedMaxVel.getX(), 0).rotate(-rotatedHeading).plus(rotatedCurVel),
                    new Vector2d(0, adjustedMaxVel.getY()).rotate(-rotatedHeading).plus(rotatedCurVel),
                    new Vector2d(-adjustedMaxVel.getX(), 0).rotate(-rotatedHeading).plus(rotatedCurVel),
                    new Vector2d(0, -adjustedMaxVel.getY()).rotate(-rotatedHeading).plus(rotatedCurVel)
            );
        } else if (adjustedMaxVel.getX()>maxVelByAccel.getX() && adjustedMaxVel.getY()>maxVelByAccel.getY()) {
            bestX = findIdealVelocity(
                    new Vector2d(maxVelByAccel.getX(), 0).rotate(-rotatedHeading).plus(rotatedCurVel),
                    new Vector2d(0, maxVelByAccel.getY()).rotate(-rotatedHeading).plus(rotatedCurVel),
                    new Vector2d(-maxVelByAccel.getX(), 0).rotate(-rotatedHeading).plus(rotatedCurVel),
                    new Vector2d(0, -maxVelByAccel.getY()).rotate(-rotatedHeading).plus(rotatedCurVel)
            );
        } else {
            bestX = Math.min(
                    findIdealVelocity(
                            new Vector2d(adjustedMaxVel.getX(), 0).rotate(-rotatedHeading).plus(rotatedCurVel),
                            new Vector2d(0, adjustedMaxVel.getY()).rotate(-rotatedHeading).plus(rotatedCurVel),
                            new Vector2d(-adjustedMaxVel.getX(), 0).rotate(-rotatedHeading).plus(rotatedCurVel),
                            new Vector2d(0, -adjustedMaxVel.getY()).rotate(-rotatedHeading).plus(rotatedCurVel)
                    ),
                    findIdealVelocity(
                            new Vector2d(maxVelByAccel.getX(), 0).rotate(-rotatedHeading).plus(rotatedCurVel),
                            new Vector2d(0, maxVelByAccel.getY()).rotate(-rotatedHeading).plus(rotatedCurVel),
                            new Vector2d(-maxVelByAccel.getX(), 0).rotate(-rotatedHeading).plus(rotatedCurVel),
                            new Vector2d(0, -maxVelByAccel.getY()).rotate(-rotatedHeading).plus(rotatedCurVel)
                    )
            );
        }

        Vector2d linearVelocities = new Vector2d(bestX, 0).rotate(offsetAngle);




*
* private double findIdealVelocity(Vector2d a, Vector2d b, Vector2d c, Vector2d d) {
        //a list containing the vertices of a rhombus describing the velocities that the robot can achieve at a given angle
        //the x-axis the direction the robot needs to travel and we want to find the largest number on the x-axis that is inside the rhombus
        List<Vector2d> velocityBounds = new ArrayList<>();

        velocityBounds.add(a);
        velocityBounds.add(b);
        velocityBounds.add(c);
        velocityBounds.add(d);

        //
        List<Integer> above = new ArrayList<>();
        List<Integer> below = new ArrayList<>();
        List<Integer> on = new ArrayList<>();

        for (int i = 0; i<4; i++) {
            if (velocityBounds.get(i).getY()>0) {
                above.add(i);
            } else if (velocityBounds.get(i).getY()<0) {
                below.add(i);
            } else {
                on.add(i);
            }
        }

        double furthestX;

        if (on.size() != 0) {
            if (on.size() == 1) {
                furthestX = velocityBounds.get(on.get(0)).getX();
            } else {
                furthestX = Math.max(velocityBounds.get(on.get(0)).getX(), velocityBounds.get(on.get(1)).getX());
            }
        } else if (above.size() == 0 || below.size() == 0) {
            furthestX = 0;
            //find the point closest to the x axis
        } else if (above.size() == 1) {
            furthestX = intersectingLinesXIntercept(
                    velocityBounds.get(above.get(0)),
                    velocityBounds.get(bounds(above.get(0)-1)),
                    velocityBounds.get(bounds(above.get(0)+1))
            );
        } else if (below.size() == 1) {
            furthestX = intersectingLinesXIntercept(
                    velocityBounds.get(below.get(0)),
                    velocityBounds.get(bounds(below.get(0)-1)),
                    velocityBounds.get(bounds(below.get(0)+1))
            );
        }
        //if it gets to here we know that their must be 2 parallel lines
        else if (above.get(0) + above.get(1) == 1 || below.get(0) + below.get(1) == 1) {
            furthestX = parallelLinesXIntercept(
                    velocityBounds.get(0),
                    velocityBounds.get(3),
                    velocityBounds.get(1),
                    velocityBounds.get(2)
            );
        } else {
            furthestX = parallelLinesXIntercept(
                    velocityBounds.get(0),
                    velocityBounds.get(1),
                    velocityBounds.get(2),
                    velocityBounds.get(3)
            );
        }
    }
    *
    *
* private double intersectingLinesXIntercept(Vector2d sharedPoint, Vector2d pointA, Vector2d pointB) {
        return parallelLinesXIntercept(sharedPoint, pointA, sharedPoint, pointB);
    }

    private double parallelLinesXIntercept(Vector2d line1a, Vector2d line1b, Vector2d line2a, Vector2d line2b) {
        return Math.max(lineXIntercept(line1a, line1b), lineXIntercept(line2a, line2b));
    }

    private double lineXIntercept(Vector2d lineA, Vector2d lineB) {
        return -lineA.getY()*((lineA.getX()-lineB.getX())/(lineA.getY()-lineB.getY()))+lineA.getX();
    }

    private int bounds(int bound) {
        if (bound<0) {
            bound += 4;
        } else if (bound>3) {
            bound -= 4;
        }
        return bound;
    }
* */
