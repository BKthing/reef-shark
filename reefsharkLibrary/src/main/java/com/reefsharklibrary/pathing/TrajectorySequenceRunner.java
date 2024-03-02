package com.reefsharklibrary.pathing;

import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.misc.ElapsedTimer;

public class TrajectorySequenceRunner {
    Rotation currentRotation = new Rotation(0, 2*Math.PI);
    private final PIDCoeficients lateralPID;
    private final PIDCoeficients headingPID;


    private ElapsedTimer runTime = new ElapsedTimer();


    public TrajectorySequenceRunner(PIDCoeficients lateralPID, PIDCoeficients headingPID) {
        this.lateralPID = lateralPID;
        this.headingPID = headingPID;
    }



}
