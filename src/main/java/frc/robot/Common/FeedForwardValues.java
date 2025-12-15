package frc.robot.Common;

public class FeedForwardValues{
    private double ks;
    private double kg;
    private double kv;
    private double ka;

    public FeedForwardValues(double ks, double kg, double kv, double ka){
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
    }

    public double getKS(){
        return ks;
    }
    public double getKG(){
        return kg;
    }
    public double getKV(){
        return kv;
    }
    public double getKA(){
        return ka;
    }
}