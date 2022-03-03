
package frc.robot.control;

/**
 * Pid
 */
public class PID {
    private final double P;
    private final double I;
    private final double i_zone;
    private final double D;
    private final double F;

    private double prev_err;
    private double i_state;

    public PID(double p, double i, double iz, double d, double f) {
        this.P = p;
        this.I = i;
        this.i_zone = iz;
        this.D = d;
        this.F = f;
        this.i_state = 0;
        this.prev_err = 0;
    }

    public double getP() {
        return this.P;
    }


    public double getI() {
        return this.I;
    }


    public double getI_zone() {
        return this.i_zone;
    }


    public double getD() {
        return this.D;
    }


    public double getF() {
        return this.F;
    }


    public double getPrev_err() {
        return this.prev_err;
    }

    public void setPrev_err(double prev_err) {
        this.prev_err = prev_err;
    }

    public double getI_state() {
        return this.i_state;
    }

    public void setI_state(double i_state) {
        this.i_state = i_state;
    }


    @Override
    public boolean equals(Object o) {
        if (o == this)
            return true;
        if (!(o instanceof PID)) {
            return false;
        }
        PID pID = (PID) o;
        return P == pID.P && I == pID.I && i_zone == pID.i_zone && D == pID.D && F == pID.F && prev_err == pID.prev_err && i_state == pID.i_state;
    }

    @Override
    public String toString() {
        return "{" +
            " P='" + getP() + "'" +
            ", I='" + getI() + "'" +
            ", i_zone='" + getI_zone() + "'" +
            ", D='" + getD() + "'" +
            ", F='" + getF() + "'" +
            ", prev_err='" + getPrev_err() + "'" +
            ", i_state='" + getI_state() + "'" +
            "}";
    }

}