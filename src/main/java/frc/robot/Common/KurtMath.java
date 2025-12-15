package frc.robot.Common;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class KurtMath {
    public static double kurtAngle(double x0, double y0, double x1, double y1){
        double angleRadians = Math.atan2(x1 - x0, y1 - y0);
        return (Math.PI*2) - (angleRadians > 0 ? angleRadians : (angleRadians+2*Math.PI));
        //return angleRadians;
    }
    public static Point addXYToPoint(Point point, double x, double y){
        Point newPoint = new Point(point.x + x, point.y + y, point.degrees, point.error, point.velocity);
        return newPoint;
    }

    public static double DistBetweenPoints(Point point0, Point point1){
        if (point0 == null || point1 == null) {
            throw new IllegalArgumentException("Points cannot be null");
        }
        double xError = point1.getX() - point0.getX();
        double yError = point1.getY() - point0.getY();
        return Math.sqrt((yError*yError)+(xError*xError));
    }

    public static double DistBetweenPose(Pose2d point0, Pose2d point1){
        if (point0 == null || point1 == null) {
            throw new IllegalArgumentException("Poses cannot be null");
        }
        double xError = point1.getX() - point0.getX();
        double yError = point1.getY() - point0.getY();
        return Math.sqrt((yError*yError)+(xError*xError));
    }

    public static Pose2d averageTwoPoses(Pose2d pose0, Pose2d pose1){
        return new Pose2d(avg2D(pose0.getX(),pose1.getX()),avg2D(pose0.getY(),pose1.getY()), new Rotation2d());
    }

    static double avg2D(double a, double b){
        return (a+b)/2;
    }

    public static Point nearestPointInList(Point[] points, Point point0){
        ArrayList<Double> distances = new ArrayList<Double>();
        double smallestDist = DistBetweenPoints(points[0], point0);
        for(Point point : points){
            double tempDist = DistBetweenPoints(point, point0);
            distances.add(tempDist);
            if(tempDist < smallestDist){
                smallestDist = tempDist;
            }
        }
        return points[distances.indexOf(smallestDist)];
    }

    public static Point LinearInterpolation(Point Start, Point End, float t){
        return new Point(Start.x + (End.x - Start.x) * t,Start.y + (End.y - Start.y) * t,Start.degrees,Start.error);
    }
    
    public static Point BezierInterpolation(Point p0, Point p1, Point p2, float t){
        Point intermediateA = LinearInterpolation(p0, p1, t);
        Point intermediateB = LinearInterpolation(p1, p2, t);
        Point pointOnCurve = LinearInterpolation(intermediateA, intermediateB, t);
        return pointOnCurve;
    }

    static Point CubicBezierInterpolation(Point p0, Point p1, Point p2, Point p3, float t){
        Point intermediateA = LinearInterpolation(p0, p1, t);
        Point intermediateB = LinearInterpolation(p1, p2, t);
        Point pointOnCurve0 = LinearInterpolation(intermediateA, intermediateB, t);
        Point intermediateC = LinearInterpolation(p2, p3, t);
        Point pointOnCurve1 = LinearInterpolation(intermediateB, intermediateC, t);
        Point pointOnCurve = LinearInterpolation(pointOnCurve0, pointOnCurve1, t);
        return pointOnCurve;
    }
    
    public static Point[] combinePoints(List<Point[]> points){
        List<Point> pointList = new ArrayList<Point>();
        for(Point[] point : points){
            for(int i = 0; i < point.length; i++){
                pointList.add(point[i]);
            }
        }
        return pointList.toArray(new Point[pointList.size()]);
    }

    public static double[] modifyAngle(double[] point, double newAngle){
        double[] newPoint = {point[0], point[1], newAngle, point[3]};
        return newPoint;
    }

    public static double[] modifyError(double[] point, double newError){
        double[] newPoint = {point[0], point[1], point[2], newError};
        return newPoint;
    }

    public static double[] addXYToPoint(double[] point, double x, double y){
        double[] newPoint = {point[0] + x, point[1] + y, point[2], point[3]};
        return newPoint;
    }

    public static double[] convertToVelocity(double[] point, double velocity, double error){
        double[] newPoint = {point[0], point[1], point[2], error, velocity};
        return newPoint;
    }

    public static ArrayList<Pose2d> convertPointListToPose2d(ArrayList<Point> points){
        ArrayList<Pose2d> poses = new ArrayList<>();
        for(Point point : points){
            poses.add(point.convertToPose2d());
        }
        return poses;
    }

    public static boolean withinTolerance(double measure, double target, double allowed){
        if(Math.abs(target - measure) <= allowed){
            return true;
        }
        return false;
    }
}
