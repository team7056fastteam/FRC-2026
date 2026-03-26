package frc.robot.Common;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveHeadingController {
    public enum HeadingType{OFF, STABILIZE, SNAP}

    HeadingType headingType = HeadingType.SNAP;

    private double targetRadians = 0;

    PIDController snapController;
    PIDController stabilizeController;

    public SwerveHeadingController(){
        double kp = 14;
        double ki = 2.7;
        double kd = 0;
        SmartDashboard.putNumber("kP", kp);
        SmartDashboard.putNumber("kI", ki);
        SmartDashboard.putNumber("kD", kd);
        snapController = new PIDController(8, 0, 0); //8, 1, 0.2
        stabilizeController = new PIDController(3.5, 0, 0.5);

        snapController.enableContinuousInput(-Math.PI, Math.PI);
        stabilizeController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setTarget(double radians){
        targetRadians = radians;
    }

    public void setState(HeadingType type){
        headingType = type;
    }

    public double calculate(double currentRadians){
        double error = MathUtil.angleModulus(targetRadians - currentRadians);

        double correction = 0;
        switch (headingType) {
            case OFF:
                break;
            case STABILIZE:
                correction = stabilizeController.calculate(0,error);
                break;
            case SNAP:
                correction = snapController.calculate(0,error);
                break;
        }
        correction = MathUtil.clamp(correction, -Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
        return correction;
    }
    public void updatePIDS(){
        snapController = new PIDController(SmartDashboard.getNumber("kP", 0), SmartDashboard.getNumber("kI", 0), SmartDashboard.getNumber("kD", 0));
    }
}
