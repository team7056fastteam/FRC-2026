package frc.robot.Common;

import java.util.ArrayList;

public class Curve {
    ArrayList<Point> points = new ArrayList<Point>();
    int resolution = 20;
    Point tempPoint = new Point(0, 0, 0, 0);

    public Curve(Point startPoint, Point endPoint, Point constraintPoint0, Point constraintPoint1){
        resolution = (int)KurtMath.DistBetweenPoints(startPoint, endPoint) * 2;
        points.add(startPoint);
        for(int i = 0; i < resolution; i++){
            float t = (i + 1f)/ resolution;
            tempPoint = KurtMath.CubicBezierInterpolation(startPoint, constraintPoint0, constraintPoint1, endPoint, t);
            double rotationT = t * 1.15;
            rotationT = rotationT > 1 ? 1 : rotationT;
            if((endPoint.getDegrees()-startPoint.getDegrees()) > 180){
                double stunnedDegrees = startPoint.getDegrees() - (((360-endPoint.getDegrees())+startPoint.getDegrees()) * rotationT);
                stunnedDegrees = stunnedDegrees < 0 ? 360+stunnedDegrees : stunnedDegrees;
                tempPoint.setDegrees(stunnedDegrees);
            }
            else if((endPoint.getDegrees()-startPoint.getDegrees()) < -180){
                double superStunnedDegrees = startPoint.getDegrees() + (((360-startPoint.getDegrees())+endPoint.getDegrees())* rotationT);
                superStunnedDegrees = superStunnedDegrees > 360 ? superStunnedDegrees-360 : superStunnedDegrees;
                tempPoint.setDegrees(superStunnedDegrees);
            }
            else{
                tempPoint.setDegrees(startPoint.getDegrees() + ((endPoint.getDegrees()-startPoint.getDegrees()) * rotationT));
            }
            
            double tempVelocity = (startPoint.getV() + ((endPoint.getV()-startPoint.getV())*t));
            tempPoint.setVelocity(tempVelocity < 5 ? tempVelocity : 5);
            points.add(tempPoint);
        }
    }
    public Point[] getPoints(){
        return points.toArray(new Point[points.size()]);
    }
    public ArrayList<Point> getArrayPoints(){
        return points;
    }
}