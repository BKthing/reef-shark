package com.reefsharklibrary.data;

public final class PIDCoeficients {



    private final double p;
    private final double i;
    private final double d;

    /**
     * Contains values for a pid algorithm to use
     *
     * @param p proportional
     * @param i integral
     * @param d derivative
     */
    public PIDCoeficients(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    /**
     * @return p returns the p value specified for the function
     * */
    public double getP() {
        return p;
    }

    /**
     * @return p returns the p value specified for the function
     * */
    public double getI() {
        return i;
    }

    /**
     * @return p returns the p value specified for the function
     * */
    public double getD() {
        return d;
    }


}
