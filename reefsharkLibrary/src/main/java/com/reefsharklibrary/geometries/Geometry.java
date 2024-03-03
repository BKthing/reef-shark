package com.reefsharklibrary.geometries;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;

import java.util.List;

public interface Geometry {
    Vector2d getPoint(double distance);

    double tangentAngle(double distance);

    double getTotalDistance();

    Vector2d startPoint();
    Vector2d endPoint();

}
