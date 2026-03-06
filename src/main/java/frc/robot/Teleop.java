package frc.robot; 

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Common.ControllerFunction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.Kicker.KickerState;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Subsystems.Spindexer.SpindexerState;
import frc.robot.Subsystems.SwerveSubsystem.SwerveState;
import frc.robot.Constants.FieldConstants;
import frc.robot.Common.SwerveHeadingController;
import frc.robot.Common.SwerveHeadingController.HeadingType;

public class Teleop {
    SwerveSubsystem _drive;
    Intake _intake;
    Spindexer _spindexer;
    Kicker _kicker;
    Shooter _shooter;
    Odometry _odometry;

    XboxController driver = new XboxController(0);
    XboxController operator = new XboxController(1);

    ControllerFunction get;
    double xT = 1, rT = 1, driveX, driveY, driveZ;

    double xPowerOffset;
    double yPowerOffset;
    double zPowerOffset;

    Translation2d adjustment = new Translation2d();

    SwerveHeadingController headingController = new SwerveHeadingController();

    boolean slowSpindexer = false;

    Alliance alliance;

    Timer timer = new Timer();


    public void TeleopInit() {
        _drive = Robot.getSwerveInstance();
        _intake = Robot.getIntakeInstance();
        _kicker = Robot.getKickerInstance();
        _shooter = Robot.getShooterInstance();
        _spindexer = Robot.getSpindexerInstance();
        _odometry = Robot.getOdometryInstance();

        get = new ControllerFunction(driver, operator);

        alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        _drive.setState(SwerveState.TeleOp);

        //don't think you need this but it was breaking
        _drive.feedSwerveSpeeds(new ChassisSpeeds(0, 0, 0));
        _drive.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0))
        });
    }


    public void Driver() {
        if (!driver.isConnected()) return;

        // turbo
        get.isPressed(get.speedAdjustment(), () -> xT = 1.4);
        
        get.isNotPressed(List.of(get.speedAdjustment(), get.negativeTurbo()), () -> xT = 0.675);

        // reorient
        get.isPressed(driver.getAButton(), () -> Robot.setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0))));

        // joystick input
        driveX = get.driverX();
        driveY = get.driverY();
        driveZ = get.driverZ();

        // apply deadband
        driveX = Math.abs(driveX) > DriveConstants.kDeadband ? driveX : 0.0;
        driveY = Math.abs(driveY) > DriveConstants.kDeadband ? driveY : 0.0;
        driveZ = Math.abs(driveZ) > DriveConstants.kDeadband ? driveZ : 0.0;

        // drive constants
        driveX *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * xT;
        driveY *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * xT;
        driveZ *= DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * rT;

        //negative turbo
        get.isPressed(get.negativeTurbo(), () -> {
            if(Math.abs(driveX) > DriveConstants.kDeadband || Math.abs(driveY) > DriveConstants.kDeadband){
                double angle = Math.atan2(get.driverY(),get.driverX());
                driveX = 0.2 * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * Math.cos(angle);
                driveY = 0.2 * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * Math.sin(angle);
            }
        });

        // flip for red alliance
        if (alliance == Alliance.Red) {
            driveX = -driveX;
            driveY = -driveY;
        }

        // auto-orient to hub
        get.isPressed(get.autoOrient(), () -> {
            Rotation2d targetRot;
            if(pastHub()){
                targetRot = getHubTargetRotation();
            } else {
                targetRot = getPassTargetRotation();
            }
            double currentAngle = Robot.getGyroscopeRotation2d().getRadians();

            headingController.setState(HeadingType.SNAP);
            headingController.setTarget(targetRot.getRadians());
            double correction = headingController.calculate(currentAngle);

            double maxAngular = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            zPowerOffset = Math.max(-maxAngular, Math.min(correction, maxAngular));
        });

        get.isNotPressed(get.autoOrient(), ()->{
            zPowerOffset = 0;
            headingController.setState(HeadingType.OFF);
        });

        // feed swerve
        _drive.feedSwerveSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(driveX + xPowerOffset, driveY - yPowerOffset, driveZ + zPowerOffset),
                Robot.getGyroscopeRotation2d()
            )
        );
    }

    public void Operator() {
        if (!operator.isConnected()) return;

        get.isPressed(get.Ingest(), () -> _intake.setState(IntakeState.Forward));
        get.isPressed(get.Outgest(), () -> _intake.setState(IntakeState.Backward));
        get.isPressed(get.IngestSlow(), () -> _intake.setState(IntakeState.ForwardSlow));
        get.isNotPressed(List.of(get.Outgest(), get.Ingest(), get.IngestSlow(), get.Shoot()), () -> _intake.setState(IntakeState.Idle));

        get.isPressed(get.UnstuckalateKicker(), () -> _kicker.setState(KickerState.Unstuckalate));

        get.isPressed(get.SpindexerSlow(), () -> slowSpindexer = true);
        get.isNotPressed(get.SpindexerSlow(), () -> slowSpindexer = false);

        get.isPressed(get.RevShooter(), () -> _shooter.fire());       
        
        get.isPressed(get.Shoot(), () -> {
            if(timer.get() < 0.3){
                _intake.setState(IntakeState.ForwardSlow);
            }
            else if(timer.get() < 0.45){
                _intake.setState(IntakeState.Backward);
            }
            else{
                timer.reset();
            }
        });

        get.isNotPressed(get.Shoot(), () -> {
            timer.reset();
            timer.start();
        });

        get.isPressed(get.Shoot(), () -> {
            _shooter.fire();
            if(_shooter.atSpeed()){
                _kicker.setState(KickerState.Firing);
                if(slowSpindexer){
                    _spindexer.setState(SpindexerState.ForwardSlow);
                } else{
                    _spindexer.setState(SpindexerState.Forward);
                } 
            }
            get.driverRumble();
        });

        get.isPressed(get.Pass(), () -> {
            _shooter.setState(ShooterState.Passing);
            if(_shooter.atSpeed()){
                _kicker.setState(KickerState.Firing);
                if(slowSpindexer){
                    _spindexer.setState(SpindexerState.ForwardSlow);
                } else{
                    _spindexer.setState(SpindexerState.Forward);
                } 
            }
            get.driverRumble();
        });

        get.isNotPressed(List.of(get.Pass(), get.Shoot()), () -> {
            if(slowSpindexer){
                _spindexer.setState(SpindexerState.ForwardSlow);
            } else {
                _spindexer.setState(SpindexerState.Idle);
            }
            get.driverUnRumble();
        });

        get.isNotPressed(List.of(get.Pass(), get.Shoot(), get.RevShooter()), () -> _shooter.setState(ShooterState.Idle));
        get.isNotPressed(List.of(get.Pass(), get.Shoot(), get.UnstuckalateKicker()), () -> _kicker.setState(KickerState.Idle));

        get.isPressed(get.AutoTargeting(), () -> _shooter.setIntendedState(ShooterState.Targeting));
        get.isPressed(get.OverrideLongShot(), () -> _shooter.setIntendedState(ShooterState.Far));
        get.isPressed(get.OverrideMidShot(), () -> _shooter.setIntendedState(ShooterState.Mid));
        get.isPressed(get.OverrideShortShot(), () -> _shooter.setIntendedState(ShooterState.Close));
    }

    public void Dashboard() {
        SmartDashboard.putBoolean("Driver Connected?", driver.isConnected());
        SmartDashboard.putBoolean("Operator Connected?", operator.isConnected());
    }

    // get rotation to hub
    private Rotation2d getHubTargetRotation() {
        Pose2d shooterPose = _odometry
            .getPose()
                .transformBy(
                    new Transform2d(
                        ShooterConstants.ShooterPoseOffsetX,
                        ShooterConstants.ShooterPoseOffsetY,
                        new Rotation2d()
                    ));

        Translation2d hubPos = (alliance == Alliance.Blue ?
                        FieldConstants.hubPosBlue : FieldConstants.hubPosRed);

        double distance = hubPos.minus(shooterPose.getTranslation()).getNorm();

        ChassisSpeeds speeds = Robot.getOdometryInstance().getFieldRelativeSpeeds();

        Translation2d linearVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        //has to match calculateRPM()
        double flightTime = (distance * ShooterConstants.FlightTimeSlope) + ShooterConstants.FlightTimeYInt;

        Translation2d leadOffset = linearVelocity.times(flightTime);

        Translation2d compensatedHubPos = hubPos.minus(leadOffset);

        Translation2d compensatedVector = compensatedHubPos.minus(shooterPose.getTranslation());

        Rotation2d angleToHub = compensatedVector.getAngle();

        //intake is front of the robot, shooter is 90 degree offset
        return angleToHub.minus(Rotation2d.fromDegrees(90)); 
    }

    public Rotation2d getPassTargetRotation(){
        Pose2d shooterPose = _odometry
            .getPose()
                .transformBy(
                    new Transform2d(
                        ShooterConstants.ShooterPoseOffsetX,
                        ShooterConstants.ShooterPoseOffsetY,
                        new Rotation2d()
                    ));
        
        Translation2d passPos;

        Translation2d passPosLeft = (alliance == Alliance.Blue ?
                        FieldConstants.passLeftPosBlue : FieldConstants.passLeftPosRed);

        Translation2d passPosRight = (alliance == Alliance.Blue ?
                        FieldConstants.passRightPosBlue : FieldConstants.passRightPosRed);

        if (shooterPose.getTranslation().getSquaredDistance(passPosRight) > shooterPose.getTranslation().getSquaredDistance(passPosLeft)){
            passPos = (alliance == Alliance.Blue ?
                        FieldConstants.passLeftPosBlue : FieldConstants.passLeftPosRed);
        } else {
            passPos = (alliance == Alliance.Blue ?
                        FieldConstants.passRightPosBlue : FieldConstants.passRightPosRed);
        }

        //use this to find angle of the vector
        Translation2d toTarget = passPos.minus(shooterPose.getTranslation());

        //shooter isn't front of the robot
        return toTarget.getAngle().minus(Rotation2d.fromDegrees(90));
    }

    public boolean pastHub(){
        if(alliance == Alliance.Blue){
            if (_odometry.getPose().getX() > FieldConstants.hubPosBlue.getX()) {
                return true;
            } else return false;
        } else if(_odometry.getPose().getX() < FieldConstants.hubPosRed.getX()){
            return true;
        } return false;
    }
}
