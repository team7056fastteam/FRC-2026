package frc.robot.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Robot;
import frc.robot.Common.FastCommand;
import frc.robot.Common.FastHolonomicController;
import frc.robot.Common.Point;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveSubsystem;

public class GoToPoint extends FastCommand{
    SwerveSubsystem _swerve;

    FastHolonomicController swerveDriveController;
    PIDController xController;
    PIDController yController;
    ProfiledPIDController thetaController;

    Point selectedPoint;
    Boolean IsComplete = false;

    SubsystemManager _manager;

    public GoToPoint(Point point){
        this.selectedPoint = point;
        IsComplete = false;
    }

    @Override
    public void init() {
        // _swerve = _manager.getSwerveInstance();

        xController = new PIDController(AutoConstants.kPXController, 0, 0);
        yController = new PIDController(AutoConstants.kPYController, 0, 0);
        //swerveDriveController = new FastHolonomicController(xController, yController, thetaController);

        IsComplete = false;
        System.out.println("Init");
    }

    @Override
    public void run() {
        // ChassisSpeeds swerveSpeeds = swerveDriveController.calculate(Robot.getPose(), selectedPoint);
        // if(Robot.isSimulation()){
        //     Robot.ModifyPoseFromSpeed(-swerveSpeeds.vyMetersPerSecond,swerveSpeeds.vxMetersPerSecond,swerveSpeeds.omegaRadiansPerSecond);
        // }
        // _swerve.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(swerveSpeeds));

        // if(swerveDriveController.atReference(Robot.getPose(), selectedPoint.setError(2.5).setVelocity(1))){
        //     IsComplete = true;
        // }  
    }

    @Override
    public Boolean isFinished() {
        _swerve.stop();
        return IsComplete;
    }

    @Override
    public void end() {
        _swerve.stop();
    }
}
