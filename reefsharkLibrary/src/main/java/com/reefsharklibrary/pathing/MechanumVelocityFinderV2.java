package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;


import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class MechanumVelocityFinderV2 {
    private final List<Pose2d> positions;

    private List<Pose2d> deltas;

    private List<Double> xZeros;
    private List<Double> yZeros;




    public MechanumVelocityFinderV2(List<Pose2d> positions, ConstraintSet constraints) {
        this.positions = positions;


        //first number is the wheels position on the path, second is which wheel on the robot
    }

    public List<Pose2d> generateVelocities() {
        deltas = findDeltas(positions);

        xZeros = findZeros((i) -> deltas.get(i).getX(), deltas.size());
        yZeros = findZeros((i) -> deltas.get(i).getY(), deltas.size());



        return new ArrayList<>();
    }


    //finds distance robot has to travel to go from prev point to this point
    private List<Pose2d> findDeltas(List<Pose2d> positions) {
        List<Pose2d> deltas = new ArrayList<>();

        deltas.add(new Pose2d(0, 0, 0));

        for (int i = 1; i<positions.size()-1; i++) {
            deltas.add(positions.get(i).minus(positions.get(i-1)));
        }

        return deltas;
    }

    private List<Double> findZeros(Function<Integer, Double> deltas, int size) {
        List<Double> zeros = new ArrayList<>();

        if (Math.signum(deltas.apply(1)) != 0) {
            zeros.add(0.0);
        }

        for (int i = 1; i<size-1; i++) {
            //Math.signum(deltas.apply(i-1)) != 0 prevents duplicate points
            if (Math.signum(deltas.apply(i)) != Math.signum(deltas.apply(i-1)) && Math.signum(deltas.apply(i-1)) != 0) {
                if (deltas.apply(i) == 0) {
                    //zero is on the number
                    zeros.add((double) i);
                } else {
                    //zero is between the numbers
                    zeros.add(i-.5);
                }
            }
        }

        return zeros;
    }

}
