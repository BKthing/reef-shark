package com.reefsharklibrary.localizers;

import com.reefsharklibrary.data.Point;
import com.reefsharklibrary.data.Pose2d;

public class CluelessConstantAccelMath {
    public static final double FIDELITY = 1E-8;

    private double lastLoop = 0.008;
    private Pose2d lastRelativeDelta = new Pose2d(0,0,0);

    public Pose2d calculate(double loopTime, Pose2d relDelta, Pose2d currPose){
        double relDeltaX = relDelta.getX();
        double relDeltaY = relDelta.getY();
        double deltaHeading = relDelta.getHeading();

        double arx = (relDeltaX*lastLoop - lastRelativeDelta.getX()*loopTime)/(loopTime*lastLoop*lastLoop + loopTime*loopTime*lastLoop);
        double vrx = relDeltaX/loopTime - arx*loopTime;
        //v_x = vrx + arx*t
        double ary = (relDeltaY*lastLoop - lastRelativeDelta.getY()*loopTime)/(loopTime*lastLoop*lastLoop + loopTime*loopTime*lastLoop);
        double vry = relDeltaY/loopTime - ary*loopTime;
        //v_y = vry + ary*t
        double arh = (deltaHeading*lastLoop - lastRelativeDelta.getHeading()*loopTime)/(loopTime*lastLoop*lastLoop + loopTime*loopTime*lastLoop);
        double vrh = deltaHeading/loopTime - arh*loopTime;
        //h = h1 + vry*t + ary*t^2



        AdaptiveQuadrature xQuadrature = new AdaptiveQuadrature(new double[] {vrx,2*arx},new double[] {currPose.getHeading(),vrh,arh});
        AdaptiveQuadrature yQuadrature = new AdaptiveQuadrature(new double[] {vry,2*ary},new double[] {currPose.getHeading(),vrh,arh});


        lastRelativeDelta = relDelta;
        lastLoop = loopTime;

        return new Pose2d(currPose.getX() + xQuadrature.evaluateCos(FIDELITY, 0, loopTime, 0) - yQuadrature.evaluateSin(FIDELITY, 0, loopTime, 0),
                currPose.getY() + yQuadrature.evaluateCos(FIDELITY, 0, loopTime, 0) + xQuadrature.evaluateSin(FIDELITY, 0, loopTime, 0),
                currPose.getHeading() + deltaHeading);
    }
}
