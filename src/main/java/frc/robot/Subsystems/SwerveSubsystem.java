package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Common.FastSubsystemBase;
import frc.robot.Common.FastTrajectory;
import frc.robot.Common.PurePursuitController;
import frc.robot.Common.SwerveHeadingController;
import frc.robot.Common.SwerveModule;
import frc.robot.Common.SwerveHeadingController.HeadingType;
import frc.robot.Constants.*;
import frc.robot.Constants;
import frc.robot.KurtLogger;
import frc.robot.KurtLogger.logType;

public class SwerveSubsystem extends FastSubsystemBase{
    private SwerveState state = SwerveState.Idle;

    private KurtLogger logger;

    private ChassisSpeeds inputSpeeds = new ChassisSpeeds(0, 0, 0);

    private FastTrajectory currentTraj = null;
    private PurePursuitController mController;

    private SwerveHeadingController mHeadingController;

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
        logger.logData(logType.event, state.toString(), "SwerveSubsystem");
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
                double x = speeds.vxMetersPerSecond;
				double y = speeds.vyMetersPerSecond;
				double theta = mHeadingController.calculate(Robot.getPose().getRotation().getRadians());
                this.inputSpeeds = new ChassisSpeeds(x,y,theta);
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

    @Override
    public void Init(KurtLogger logger) {
        this.logger = logger;
        runChassis(0, 0, 0);
        mController = new PurePursuitController();
        mHeadingController = new SwerveHeadingController();
    }
    @Override
    public void run() {
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
                updatePathFollower();
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

    public SwerveState getState(){
        return state;
    }

    @Override
    public void stop() {
        logger = null;
        runChassis(0, 0, 0);
    }

    public void setTrajectory(FastTrajectory trajectory){
        if(trajectory == null) return;
        setState(SwerveState.Path_Following);
        mController.setTrajectory(trajectory);
        currentTraj = trajectory;
    }

    public boolean trajectoryDone(){
        if(currentTraj == null) return true; 
        return currentTraj.isDone();
    }

    void updatePathFollower(){
        if(currentTraj != null){
            inputSpeeds = mController.update(Robot.getPose());
            if(mController.isDone()){
                currentTraj.reset();
                setState(SwerveState.Idle);
                currentTraj = null;
            }
        }
        else{
            setState(SwerveState.Idle);
        }
        
    }

    @Override
    public void dashboard() {
        SmartDashboard.putString("Front Left State", frontLeft.getState().toString());
        SmartDashboard.putString("Front Right State", frontRight.getState().toString());
        SmartDashboard.putString("Back Left State", backLeft.getState().toString());
        SmartDashboard.putString("Back Right State", backRight.getState().toString());
        SmartDashboard.putString("Swerve State", state.toString());
    }
}