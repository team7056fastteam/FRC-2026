package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Common.ControllerFunction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.IntakePivot.IntakePivotState;
import frc.robot.Subsystems.Kicker.KickerState;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Subsystems.Spindexer.SpindexerState;
import frc.robot.Subsystems.SwerveSubsystem.SwerveState;

public class Teleop {
    SwerveSubsystem _drive;
    Intake _intake;
    Spindexer _spindexer;
    Kicker _kicker;
    Shooter _shooter;
    IntakePivot _intakePivot;

    XboxController driver = new XboxController(0);
    XboxController operator = new XboxController(1);

    ControllerFunction get;
    double xT = 1, rT = 1, driveX, driveY, driveZ, z;

    AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    double xPowerOffset;
    double yPowerOffset;
    
    double kTagP = 0.1;
    double poleOffset = 7;
    double robotOffset = 17.5;
    double maxTagPower = 2;
    double closeMaxTagPower = 0.85;
    double closeTagDist = 45;
    Translation2d adjustment = new Translation2d();

    Boolean up, down, right, left;

    public void TeleopInit(){
        _drive = Robot.getSwerveInstance();
        // _intake = Robot.getIntakeInstance();
        // _intakePivot = Robot.getIntakePivotInstance();
        // _kicker = Robot.getKickerInstance();
        // _shooter = Robot.getShooterInstance();
        // _spindexer = Robot.getSpindexerInstance();
        
        get = new ControllerFunction(driver, operator);

        _drive.setState(SwerveState.TeleOp);
    }

    public void Driver(){
        if(!driver.isConnected()){ return; }

        get.isPressed(get.speedAdjustment(),()-> {xT = 1.4;});
        get.isNotPressed(get.speedAdjustment(),()-> {xT = 0.675;});

        get.isNotPressed(List.of(get.driverLeftTrigger(),get.driverRightTrigger()), ()-> {
            xPowerOffset = 0;
            yPowerOffset = 0;
        });

        // get.isPressed(driver.getAButton(),() -> Robot.setPose(new Pose2d(0,0, Rotation2d.fromDegrees(0))));


        get.isPressed(get.Up() && up, ()-> {adjustment = adjustment.plus(new Translation2d(0,2)); up = false; System.out.println(adjustment.toString());});
        get.isPressed(get.Down() && down, ()-> {adjustment = adjustment.plus(new Translation2d(0,-2)); down = false;System.out.println(adjustment.toString());});
        get.isPressed(get.Right() && right, ()-> {adjustment = adjustment.plus(new Translation2d(2,0)); right = false;System.out.println(adjustment.toString());});
        get.isPressed(get.Left() && left, ()-> {adjustment = adjustment.plus(new Translation2d(-2,0)); left = false;System.out.println(adjustment.toString());});
        get.isNotPressed(get.Up(), ()-> {up = true;});
        get.isNotPressed(get.Down(), ()-> {down = true;});
        get.isNotPressed(get.Right(), ()-> {right = true;});
        get.isNotPressed(get.Left(), ()-> {left = true;});

        driveX = get.driverX();
        driveY = get.driverY();
        driveZ = get.driverZ();

        //apply deadband
        driveX = Math.abs(driveX) > DriveConstants.kDeadband ? driveX : 0.0;
        driveY = Math.abs(driveY) > DriveConstants.kDeadband ? driveY : 0.0;
        driveZ = Math.abs(driveZ) > DriveConstants.kDeadband ? driveZ : 0.0;

        //apply DriveConstants
        driveX = driveX * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * xT;
        driveY = driveY * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * xT;
        driveZ = driveZ * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * rT;

        if(DriverStation.getAlliance().get() == Alliance.Red){
            driveX = -driveX;
            driveY = -driveY;
        }

        _drive.feedSwerveSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(driveX+xPowerOffset,driveY-yPowerOffset,driveZ), Robot.getGyroscopeRotation2d()));
    }

    public void Operator(){
        if(!operator.isConnected()){ return; }

        get.isPressed(get.Ingest(), ()-> _intake.setState(IntakeState.Forward));
        get.isPressed(get.Outgest(), ()-> _intake.setState(IntakeState.Backward));
        get.isPressed(get.IngestSlow(), ()-> _intake.setState(IntakeState.ForwardSlow));
        get.isNotPressed(List.of(get.Outgest(), get.Ingest(), get.IngestSlow()), ()-> _intake.setState(IntakeState.Idle));

        get.isPressed(get.Shoot(), ()-> {_shooter.fire();_kicker.setState(KickerState.Firing);_spindexer.setState(SpindexerState.Forward);});
        get.isNotPressed(get.Shoot(), ()-> {_spindexer.setToIntendedState();_kicker.setState(KickerState.Idle);_shooter.setState(ShooterState.Idle);});

        get.isPressed(get.AutoTargeting(), ()-> _shooter.setIntendedState(ShooterState.Targeting));
        get.isPressed(get.OverrideLongShot(), ()-> _shooter.setIntendedState(ShooterState.Far));
        get.isPressed(get.OverrideMidShot(), ()-> _shooter.setIntendedState(ShooterState.Mid));
        get.isPressed(get.OverrideShortShot(), ()-> _shooter.setIntendedState(ShooterState.Close));

        get.isPressed(get.ToggleSpindexer(), ()-> _spindexer.toggleSpindexer());

        get.isPressed(get.IntakePivotToggle(), ()-> _intakePivot.togglePos());

        get.isPressed(get.IntakePivotRehome(), ()-> _intakePivot.setState(IntakePivotState.Rehoming));
    }

    // Translation2d convertWPITranslationToKurtTranslation(Translation2d initalTranslation){
    //     return new Translation2d(Units.metersToInches(8.05 - initalTranslation.getY()), Units.metersToInches(initalTranslation.getX()));
    // }

    public void Dashboard(){ 
        SmartDashboard.putBoolean("Driver Connected?", driver.isConnected());
        SmartDashboard.putBoolean("Operater Connected?", operator.isConnected());
    }
}