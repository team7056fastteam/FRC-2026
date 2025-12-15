package frc.robot.Common;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class SwerveHeadingController {
    public enum HeadingType{OFF, STABILIZE, SNAP}

    HeadingType headingType = HeadingType.SNAP;

    private double targetRadians = 0;

    PIDController snapController;
    PIDController stabilizeController;

    public SwerveHeadingController(){
        snapController = new PIDController(5, 0, 0.5);
        stabilizeController = new PIDController(3.5, 0, 0.5);

        snapController.enableContinuousInput(0, 2*Math.PI);
        stabilizeController.enableContinuousInput(0, 2*Math.PI);
    }

    public void setTarget(double radians){
        targetRadians = radians;
    }

    public void setState(HeadingType type){
        headingType = type;
    }

    public double calculate(double currentRadians){
        double correction = 0;
        switch (headingType) {
            case OFF:
                break;
            case STABILIZE:
                correction = stabilizeController.calculate(currentRadians,targetRadians);
                break;
            case SNAP:
                correction = snapController.calculate(currentRadians,targetRadians);
                break;
        }
        MathUtil.clamp(correction, -Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
        return correction;
    }
}
