package frc.robot.Common;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class FastHolonomicController {

    final static double acceptableThetaError = Math.toRadians(180);

    PIDController xController;
    PIDController yController;
    PIDController thetaController;

    Pose2d poseError;
    Boolean firstTime;

    public FastHolonomicController(PIDController xController, PIDController yController, PIDController thetaController){
        firstTime = false;
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        this.thetaController.enableContinuousInput(0, Math.PI * 2);  //Degrees : 0 to 360
    }
    public ChassisSpeeds calculate(Pose2d robotPose, FastTrajectory.State desiredState, FastTrajectory.State feedState){
        Pose2d desiredPose = desiredState.pose;

        if(!firstTime){
            //thetaController.reset(robotPose.getRotation().getRadians());
            firstTime = true;
        }

        double xFeedback = xController.calculate(robotPose.getX(), desiredPose.getX());
        double yFeedback = yController.calculate(robotPose.getY(), desiredPose.getY());
        
        double thetaFeedback = thetaController.calculate(robotPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());

        poseError = desiredPose.relativeTo(robotPose);

        return ChassisSpeeds.fromFieldRelativeSpeeds(Units.inchesToMeters(feedState.linearVelocityY*.5 + yFeedback), Units.inchesToMeters((feedState.linearVelocityX*.5 + xFeedback)*-1), thetaFeedback, robotPose.getRotation());
        //return ChassisSpeeds.fromFieldRelativeSpeeds(Units.inchesToMeters(yFeedback), Units.inchesToMeters((xFeedback)*-1), thetaFeedback, robotPose.getRotation());
    }
    public boolean atReference(Pose2d robotPose, Point desiredPoint){
        if(poseError.getX() < desiredPoint.getError() && poseError.getY() < desiredPoint.getError()){
            return true;
        }
        return false;
    }
    public boolean atReferenceAndHeading(Pose2d robotPose, Point desiredPoint){
        double xError = Math.abs(robotPose.getX() - desiredPoint.getX());
        double yError = Math.abs(robotPose.getY() - desiredPoint.getY());
        double thetaError = Math.abs(robotPose.getRotation().getRadians() - desiredPoint.getRadians());
        if(thetaError > Math.PI){
            thetaError = Math.abs(2*Math.PI - thetaError);
        }

        if(xError < desiredPoint.getError() && yError < desiredPoint.getError() && thetaError < acceptableThetaError){
            return true;
        }
        return false;
    }
}
