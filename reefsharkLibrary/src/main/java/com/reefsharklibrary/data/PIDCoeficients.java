package com.reefsharklibrary.data;

public final class PIDCoeficients {



    private final double p;
    private final double i;
    private final double d;
    private final double kV;
    private final double kA;


    /**
     * Contains values for a pid algorithm to use
     *
     * @param p proportional
     * @param i integral
     * @param d derivative
     */

    public PIDCoeficients(double p, double i, double d, double kV, double kA) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.kV = kV;
        this.kA = kA;
    }

    public PIDCoeficients(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;

        //got these default values from roadrunner, hopefully their good
        this.kV = .0128;
        this.kA = .0054;
    }

    /**
     * @return returns the p value specified for the function. Uses for proportional in PID
     * */
    public double getP() {
        return p;
    }

    /**
     * @return returns the i value specified for the function
     * */
    public double getI() {
        return i;
    }

    /**
     * @return returns the d value specified for the function
     * */
    public double getD() {
        return d;
    }

    /**
     * @return returns the kV value specified for the function
     * */
    public double getkV() {
        return kV;
    }

    /**
     * @return returns the kA value specified for the function
     * */
    public double getkA() {
        return kA;
    }

}
