package com.reefsharklibrary.data;

public final class PIDCoeficients {



    private final int p;
    private final int i;
    private final int d;

    /**
     * Contains values for a pid algorithm to use
     *
     * @param p proportional
     * @param i integral
     * @param d derivative
     */
    public PIDCoeficients(int p, int i, int d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    /**
     * @return p returns the p value specified for the function
     * */
    public int getP() {
        return p;
    }

    /**
     * @return p returns the p value specified for the function
     * */
    public int getI() {
        return i;
    }

    /**
     * @return p returns the p value specified for the function
     * */
    public int getD() {
        return d;
    }


}
