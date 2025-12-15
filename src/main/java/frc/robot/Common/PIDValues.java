package frc.robot.Common;

public class PIDValues{
    private double p;
    private double i;
    private double d;

    public PIDValues(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double getP(){
        return p;
    }
    public double getI(){
        return i;
    }
    public double getD(){
        return d;
    }
}
