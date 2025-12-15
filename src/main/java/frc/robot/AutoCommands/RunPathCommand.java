package frc.robot.AutoCommands;

import java.util.ArrayList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.KurtLogger.logType;
import frc.robot.Robot;
import frc.robot.Common.FastCommand;
import frc.robot.Common.FastTrajectory;
import frc.robot.Common.KurtMath;
import frc.robot.Common.Point;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.SwerveSubsystem.SwerveState;

public class RunPathCommand extends FastCommand{
    SwerveSubsystem _drive;

    ArrayList<Point> path = new ArrayList<>();
    FastTrajectory trajectory;

    SubsystemManager _manger = SubsystemManager.getInstance();

    Timer timeOutTimer;

    public RunPathCommand(ArrayList<Point> path){
        this.path = path;
        trajectory = new FastTrajectory(KurtMath.convertPointListToPose2d(path), AutoConstants.kMaxSpeedInchesPerSecond, AutoConstants.kMaxAccelerationInchesPerSecondSquared);
    
        timeOutTimer = new Timer();
    }

    @Override
    public void init() {
        _drive.setState(SwerveState.Path_Following);
        trajectory.reset();
        _drive.setTrajectory(trajectory);
        _drive.setState(SwerveState.Path_Following);

        Robot.getAutoLogger().logData(logType.event, "Path Started", "AutoPath");

        timeOutTimer.reset();
        timeOutTimer.start();
    }

    @Override
    public void run() {
    }

    @Override
    public Boolean isFinished() {
        return _drive.trajectoryDone() || timeOutTimer.get() > trajectory.getTrajectoryTotalTime() + 2;
    }

    @Override
    public void end() {
        Robot.getAutoLogger().logData(logType.event, "Path Finished", "AutoPath");
        _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0)));
    }
    
}
