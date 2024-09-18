package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Point;
import com.reefsharklibrary.data.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class ConstantAccelSolver implements Solver {

    List<Point> deltaX = new ArrayList<>();
    List<Point> deltaY = new ArrayList<>();
    List<Point> deltaHeading = new ArrayList<>();



    @Override
    public Pose2d getRelativeFieldMovement(Point deltaX, Point deltaY, Point deltaHeading, double heading, Pose2d prevPose) {



        return null;
    }


    private void updateList() {
        if (deltaX.size()>2) {
            deltaX.
        }
    }
}
