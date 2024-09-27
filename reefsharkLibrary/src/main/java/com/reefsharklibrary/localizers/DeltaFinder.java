package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Point;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.TimePose2d;

import java.util.List;

public interface DeltaFinder {



//    TimePose2d getDeltas();
//
//    TimePose2d getRelDeltas();

    void update(Point rawX, Point rawY, Point rawHeading);

    void clearDeltas(Point rawX, Point rawY, Point rawHeading);

    Point getDeltaX();

    Point getDeltaY();

    Point getDeltaHeading();

    double getChangeHeading();

//    Point getHeading();


}
