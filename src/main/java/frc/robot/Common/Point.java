package frc.robot.Common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Point {
    double x=0, y=0, degrees=0, error=0, velocity=0;
    /**Standard Point Constructor*/
    public Point(double x, double y, double degrees, double error){
        this.x = x;
        this.y = y;
        this.degrees = degrees;
        this.error = error;
    }
    /**Velocity Point Constructor*/
    public Point(double x, double y, double degrees, double error, double velocity){
        this.x = x;
        this.y = y;
        this.degrees = degrees;
        this.error = error;
        this.velocity = velocity;
    }
    /**Empty Point Constructor*/
    public Point(){}
    /**Simple Point Constructor*/
    public Point(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.degrees = h;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getRadians(){
        return Math.toRadians(degrees);
    }
    public double getDegrees(){
        return degrees;
    }
    public double getError(){
        return error;
    }
    public Point setX(double x){
        this.x = x;
        return this;
    }
    public Point setY(double y){
        this.y = y;
        return this;
    }
    public Point setXY(double x, double y){
        this.x = x;
        this.y = y;
        return this;
    }
    public Point setVelocity(double v){
        this.velocity = v;
        return this;
    }
    public Point addXY(double x, double y){
        this.x = this.x + x;
        this.y = this.y + y;
        return this;
    }
    public Point setDegrees(double degrees){
        this.degrees = degrees;
        return this;
    }
    public Point setRadians(double radians){
        this.degrees = Math.toDegrees(radians);
        return this;
    }
    public Point setError(double error){
        this.error = error;
        return this;
    }
    public double getV(){
        return velocity;
    }
    public String toString(){
        if(velocity > 0){
            return "X: " + x + " Y: " + y + " Heading( Radians: " + getRadians() + " Degrees: " + degrees + " )" + " Error: " + error + " Velocity: " + velocity;
        }
        return "X: " + x + " Y: " + y + " Heading( Radians: " + getRadians() + " Degrees: " + degrees + " )";
    }

    public Pose2d convertToPose2d(){
        return new Pose2d(x,y,Rotation2d.fromDegrees(degrees));
    }
}