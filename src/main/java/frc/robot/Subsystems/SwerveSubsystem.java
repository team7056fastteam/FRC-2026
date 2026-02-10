package frc.robot.Subsystems;

import frc.robot.Odometry;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Common.SwerveHeadingController;
import frc.robot.Common.SwerveModule;
import frc.robot.Common.SwerveHeadingController.HeadingType;
import frc.robot.Constants.*;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase{
    private SwerveState state = SwerveState.Idle;

    private ChassisSpeeds inputSpeeds = new ChassisSpeeds(0, 0, 0);

    private SwerveHeadingController mHeadingController;

    RobotConfig robotConfig;

    private Odometry _odometry = Robot.getOdometryInstance();

    
    public enum SwerveState{Idle, Lock_Wheels, TeleOp, Path_Following, Heading_Control, Auto_Extra}

    //sets the constants for each module
    private final static SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final static SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final static SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftTurningMotorReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final static SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightTurningMotorReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    

    //sets the states for each module
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void setState(SwerveState state){
        this.state = state;
    }

    public void runChassis(double driveX, double driveY, double driveZ){
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, driveZ, Robot.getGyroscopeRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void runChassis(ChassisSpeeds speed){
        if(Robot.isSimulation()){
            Robot.ModifyPoseFromSpeed(speed);
        }
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speed);
        setModuleStates(moduleStates);
    }

    public void snapHeading(double radians){
        setState(SwerveState.Heading_Control);
        mHeadingController.setState(HeadingType.SNAP);
        mHeadingController.setTarget(radians);
    }

    public void stabilizeHeading(double radians){
        setState(SwerveState.Heading_Control);
        mHeadingController.setState(HeadingType.STABILIZE);
        mHeadingController.setTarget(radians);
    }

    public void feedSwerveSpeeds(ChassisSpeeds speeds){
        if (state == SwerveState.Path_Following) {
			if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
					> Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * .1) {
                    state = SwerveState.TeleOp;
			} else {
				return;
			}
        }
        else if(state == SwerveState.Heading_Control){
            if (Math.abs(speeds.omegaRadiansPerSecond) > 1.0) {
                    state = SwerveState.TeleOp;
			} else {
                // double x = speeds.vxMetersPerSecond;
				// double y = speeds.vyMetersPerSecond;
				// double theta = mHeadingController.calculate(Robot.getPose().getRotation().getRadians());
                // this.inputSpeeds = new ChassisSpeeds(x,y,theta);
				return;
			}
        }
        else if(state == SwerveState.Lock_Wheels){
            if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
					> Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * .1) {
                    state = SwerveState.TeleOp;
			} else {
				return;
			}
        }
        else if(state == SwerveState.Auto_Extra){
        }
        else if(state != SwerveState.TeleOp){
            state = SwerveState.TeleOp;
        }
        this.inputSpeeds = speeds;
    }

    public SwerveSubsystem() {
        runChassis(0, 0, 0);
        // mController = new PurePursuitController();
        mHeadingController = new SwerveHeadingController();
        try{
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    }

    @Override
    public void periodic() {
        switch (state) {
            case Idle:
                runChassis(0, 0, 0);
                break;
            case TeleOp:
                runChassis(inputSpeeds);
                break;
            case Heading_Control:
                runChassis(inputSpeeds);
                break;
            case Path_Following:
                // updatePathFollower();
                runChassis(inputSpeeds);
                break;
            case Lock_Wheels:
                frontLeft.setDesiredStateUnrestricted(new SwerveModuleState(0, new Rotation2d(Math.toRadians(45))));
                frontRight.setDesiredStateUnrestricted(new SwerveModuleState(0, new Rotation2d(Math.toRadians(315))));
                backLeft.setDesiredStateUnrestricted(new SwerveModuleState(0, new Rotation2d(Math.toRadians(315))));
                backRight.setDesiredStateUnrestricted(new SwerveModuleState(0, new Rotation2d(Math.toRadians(45))));
                break;
            case Auto_Extra:
                runChassis(inputSpeeds);
                break;
        }
    }

    public Command followPathCommand(String pathName){
        try{
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return new FollowPathCommand(
                path,
                () -> _odometry.getPose(), // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                robotConfig, // The robot configuration
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
  
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public void drive(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        // PathPlanner outputs ROBOT-relative speeds
        runChassis(speeds);
    }



    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
    );
    }



    public void dashboard() {
        SmartDashboard.putString("Front Left State", frontLeft.getState().toString());
        SmartDashboard.putString("Front Right State", frontRight.getState().toString());
        SmartDashboard.putString("Back Left State", backLeft.getState().toString());
        SmartDashboard.putString("Back Right State", backRight.getState().toString());
        SmartDashboard.putString("Swerve State", state.toString());
    }
}