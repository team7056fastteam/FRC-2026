package frc.robot; 

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
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
    double xT = 1, rT = 1.25, driveX, driveY, driveZ;

    double xPowerOffset;
    double yPowerOffset;
    double zPowerOffset;

    Rotation2d targetRot;

    SwerveHeadingController headingController = new SwerveHeadingController();

    boolean slowSpindexer = false;

    Alliance alliance;

    boolean bumpButton = true;

    boolean intakeReady = false;

    boolean overrideAtSpeed = false;

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

        get.isPressed(get.lockWheels(), () -> _drive.setState(SwerveState.Lock_Wheels));
        get.isNotPressed(get.lockWheels(), () -> _drive.setState(SwerveState.TeleOp));

        // turbo
        get.isPressed(get.speedAdjustment(), () -> xT = 1.7);
        
        get.isNotPressed(List.of(get.speedAdjustment()), () -> xT = 0.675);

        // reorient
        get.isPressed(get.Reset(), () -> _odometry.zeroPigeon());

        // joystick input
        driveX = get.driverX();
        driveY = get.driverY();
        driveZ = get.driverZ();

        // apply deadband
        if(!(Math.abs(driveX) > DriveConstants.kDeadband || Math.abs(driveY) > DriveConstants.kDeadband)){
            driveX = 0;
            driveY = 0;
        }
        driveZ = Math.abs(driveZ) > DriveConstants.kDeadband ? driveZ : 0.0;

        // drive constants
        driveX *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * xT;
        driveY *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * xT;
        driveZ *= DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * rT;

        if(alliance == Alliance.Red){
            driveX = -driveX;
            driveY = -driveY;
        }

        // auto-orient to hub
        get.isPressed(get.autoOrient(), () -> {
            if(pastHub()){
                targetRot = getHubTargetRotation();
            } else {
                targetRot = getPassTargetRotation();
            }
        });

        get.isPressed(get.alignFront() && alliance == Alliance.Blue, () -> targetRot = Rotation2d.fromDegrees(0));
        get.isPressed(get.alignRight() && alliance == Alliance.Blue, () -> targetRot = Rotation2d.fromDegrees(-90));
        get.isPressed(get.alignBack() && alliance == Alliance.Blue, () -> targetRot = Rotation2d.fromDegrees(180));
        get.isPressed(get.alignLeft() && alliance == Alliance.Blue, () -> targetRot = Rotation2d.fromDegrees(90));

        get.isPressed(get.alignFront() && alliance == Alliance.Red, () -> targetRot = Rotation2d.fromDegrees(180));
        get.isPressed(get.alignRight() && alliance == Alliance.Red, () -> targetRot = Rotation2d.fromDegrees(90));
        get.isPressed(get.alignBack() && alliance == Alliance.Red, () -> targetRot = Rotation2d.fromDegrees(0));
        get.isPressed(get.alignLeft() && alliance == Alliance.Red, () -> targetRot = Rotation2d.fromDegrees(-90));

        get.isPressed(get.negativeTurbo(), () -> {
            if (bumpButton) {
                double current = _odometry.getPose().getRotation().getRadians();
                double option1;
                double option2;
                if(blueHub() || (redHalf() && !redHub())) {
                    option1 = -160;
                    option2 = 160;
                } else{
                    option1 = 20;
                    option2 = -20;
                }
                double error1 = Math.abs(MathUtil.angleModulus(Units.degreesToRadians(option1) - current));
                double error2 = Math.abs(MathUtil.angleModulus(Units.degreesToRadians(option2) - current));
                targetRot = (error1 < error2) ? Rotation2d.fromDegrees(option1) : Rotation2d.fromDegrees(option2);
                bumpButton = false;
            }
        });

        get.isNotPressed(get.negativeTurbo(), () -> bumpButton = true);

        get.isPressed(get.alignBack() || get.alignFront() || get.alignRight() || get.alignLeft() || get.autoOrient() || get.negativeTurbo(), () -> {
            double currentAngle = MathUtil.angleModulus(_odometry.getPose().getRotation().getRadians());
            headingController.updatePIDS();
            SmartDashboard.putNumber("Target Rotation", targetRot.getDegrees());
            headingController.setState(HeadingType.SNAP);
            headingController.setTarget(targetRot.getRadians());
            double correction = headingController.calculate(MathUtil.angleModulus(currentAngle));

            double maxAngular = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            zPowerOffset = MathUtil.clamp(correction, -maxAngular, maxAngular);
            // if(Math.abs(currentAngle - targetRot.getRadians()) < Math.toRadians(1)){
            //     zPowerOffset = 0;
            // }
        });

        get.isNotPressed(List.of(get.autoOrient(), get.alignBack(), get.alignFront(), get.alignLeft(), get.alignRight(), get.negativeTurbo()), ()->{
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

        get.isNotPressed(get.Shoot(), () -> {
            timer.reset();
            timer.start();
            intakeReady = false;
        });

        get.isPressed(get.Shoot(), () -> {
            _shooter.fire();
            if(_shooter.atSpeed() || overrideAtSpeed){
                    intakeReady = true;
                    _kicker.setState(KickerState.Firing);
                    if(slowSpindexer){
                        _spindexer.setState(SpindexerState.ForwardSlow);
                    } else{
                        _spindexer.setState(SpindexerState.Forward);
                    } 
                }
            if(intakeReady){
                if(DriveConstants.IntakeBackAndForthEnabled){
                    if(timer.get() < DriveConstants.IntakeForwardTime){
                        _intake.setState(IntakeState.Forward);
                    } else if(timer.get() < DriveConstants.IntakeStopTime){
                        _intake.setState(IntakeState.Idle);
                    } else if(timer.get() < DriveConstants.IntakeBackwardTime){
                        _intake.setState(IntakeState.Backward);
                    }
                    else{
                        timer.reset();
                    }
                }
            }
            get.driverRumble();
        });

        get.isPressed(get.Pass(), () -> {
            _shooter.setState(ShooterState.Passing);
            if(_shooter.atSpeed() || overrideAtSpeed){
                if(DriveConstants.IntakeBackAndForthEnabled){
                    if(timer.get() < DriveConstants.IntakeForwardTime){
                        _intake.setState(IntakeState.ForwardSlow);
                    }
                    else if(timer.get() < DriveConstants.IntakeBackwardTime){
                        _intake.setState(IntakeState.Backward);
                    }
                    else{
                        timer.reset();
                    }
                }
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

        get.isPressed(get.OverrideAtSpeed(), () -> overrideAtSpeed = true);
        get.isNotPressed(get.OverrideAtSpeed(), () -> overrideAtSpeed = false);
    }

    public void Dashboard() {
        SmartDashboard.putBoolean("Driver Connected?", driver.isConnected());
        SmartDashboard.putBoolean("Operator Connected?", operator.isConnected());
        SmartDashboard.putBoolean("Past Hub", pastHub());
    }

    // get rotation to hub
    private Rotation2d getHubTargetRotation() {
        Pose2d robotPose = _odometry.getPose();
        Translation2d shooterTranslation = robotPose.getTranslation().plus(
            new Translation2d(
                ShooterConstants.ShooterPoseOffsetX,
                ShooterConstants.ShooterPoseOffsetY
            ).rotateBy(robotPose.getRotation())
        );

        Pose2d shooterPose = new Pose2d(shooterTranslation, robotPose.getRotation());
        Translation2d hubPos = (alliance == Alliance.Blue ?
                        FieldConstants.hubPosBlue : FieldConstants.hubPosRed);

        // ChassisSpeeds speeds = Robot.getOdometryInstance().getFieldRelativeSpeeds();

        // Rotation2d robotRot = _odometry.getPose().getRotation();

        // Translation2d linearVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(robotRot);

        // double distance = hubPos.minus(shooterPose.getTranslation()).getNorm();
        // double distanceInches = Units.metersToInches(distance);

        //has to match calculateRPM()
        // double flightTime = (distanceInches * ShooterConstants.FlightTimeSlope) + ShooterConstants.FlightTimeYInt;

        // Translation2d leadOffset = linearVelocity.times(flightTime);

        // Translation2d compensatedHubPos = hubPos.minus(leadOffset);
        Translation2d compensatedHubPos = hubPos;

        double compensatedDistance = compensatedHubPos.minus(shooterPose.getTranslation()).getNorm();

        Translation2d compensatedVector = compensatedHubPos.minus(shooterPose.getTranslation());

        Rotation2d rawAngle = compensatedVector.getAngle();
        Rotation2d currentRot = _odometry.getPose().getRotation();
        Rotation2d angleToHub = currentRot.plus(
            Rotation2d.fromRadians(
                MathUtil.angleModulus(rawAngle.minus(currentRot).getRadians())
            )
        );

        SmartDashboard.putNumber("Hub Distance", compensatedDistance);

        //intake is front of the robot, shooter is 90 degree offset
        return angleToHub.minus(Rotation2d.fromDegrees(DriveConstants.shooterAngleOffset)); 
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
        return toTarget.getAngle().minus(Rotation2d.fromDegrees(DriveConstants.shooterAngleOffset));
    }

    public boolean pastHub(){
        if(alliance == Alliance.Blue){
            if (_odometry.getPose().getX() < FieldConstants.hubPosBlue.getX()) {
                return true;
            } else return false;
        } else if(_odometry.getPose().getX() > FieldConstants.hubPosRed.getX()){
            return true;
        } return false;
    }

    public boolean blueHalf(){
            if (_odometry.getPose().getX() < FieldConstants.middleOfField.getX()) {
                return true;
            }  else return false;
    }

    public boolean blueHub(){
            if (_odometry.getPose().getX() < FieldConstants.hubPosBlue.getX()) {
                return true;
            }  else return false;
    }

    public boolean redHalf(){
        if(_odometry.getPose().getX() > FieldConstants.middleOfField.getX()){
            return true;
        } return false;
        
    }

    public boolean redHub(){
        if(_odometry.getPose().getX() > FieldConstants.hubPosRed.getX()){
            return true;
        } return false;
    }
}