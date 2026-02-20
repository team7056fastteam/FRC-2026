package frc.robot; 

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Common.ControllerFunction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.IntakePivot.IntakePivotState;
// import frc.robot.Subsystems.Kicker.KickerState;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Subsystems.Spindexer.SpindexerState;
import frc.robot.Subsystems.SwerveSubsystem.SwerveState;
import frc.robot.Constants.FieldConstants;

public class Teleop {
    SwerveSubsystem _drive;
    // Intake _intake;
    // Spindexer _spindexer;
    // Kicker _kicker;
    // Shooter _shooter;
    // IntakePivot _intakePivot;

    XboxController driver = new XboxController(0);
    XboxController operator = new XboxController(1);

    ControllerFunction get;
    double xT = 1, rT = 1, driveX, driveY, driveZ;

    double xPowerOffset;
    double yPowerOffset;

    Translation2d adjustment = new Translation2d();

    Boolean up, down, right, left;

    public void TeleopInit() {
        _drive = Robot.getSwerveInstance();
        // _intake = Robot.getIntakeInstance();
        // _intakePivot = Robot.getIntakePivotInstance();
        // _kicker = Robot.getKickerInstance();
        // _shooter = Robot.getShooterInstance();
        // _spindexer = Robot.getSpindexerInstance();

        get = new ControllerFunction(driver, operator);

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
        get.isNotPressed(get.speedAdjustment(), () -> xT = 0.675);

        // reset offsets
        get.isNotPressed(List.of(get.driverLeftTrigger(), get.driverRightTrigger()), () -> {
            xPowerOffset = 0;
            yPowerOffset = 0;
        });

        // reorient
        get.isPressed(driver.getAButton(), () -> Robot.setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0))));

        // manual adjust
        get.isPressed(get.Up() && up, () -> { adjustment = adjustment.plus(new Translation2d(0, 2)); up = false; });
        get.isPressed(get.Down() && down, () -> { adjustment = adjustment.plus(new Translation2d(0, -2)); down = false; });
        get.isPressed(get.Right() && right, () -> { adjustment = adjustment.plus(new Translation2d(2, 0)); right = false; });
        get.isPressed(get.Left() && left, () -> { adjustment = adjustment.plus(new Translation2d(-2, 0)); left = false; });
        get.isNotPressed(get.Up(), () -> up = true);
        get.isNotPressed(get.Down(), () -> down = true);
        get.isNotPressed(get.Right(), () -> right = true);
        get.isNotPressed(get.Left(), () -> left = true);

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

        // flip for red alliance
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            driveX = -driveX;
            driveY = -driveY;
        }

        // auto-orient to hub
        get.isPressed(get.driverLeftTrigger(), () -> {
            Rotation2d targetRot = getHubTargetRotation();
            double currentAngle = Robot.getGyroscopeRotation2d().getRadians();
            double error = targetRot.getRadians() - currentAngle;
            error = Math.atan2(Math.sin(error), Math.cos(error)); // wrap [-pi, pi]

            double kP = 2.0; // tunable
            driveZ += error * kP;

            double maxAngular = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            driveZ = Math.max(-maxAngular, Math.min(driveZ, maxAngular));
        });

        // feed swerve
        _drive.feedSwerveSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(driveX + xPowerOffset, driveY - yPowerOffset, driveZ),
                Robot.getGyroscopeRotation2d()
            )
        );
    }

    public void Operator() {
        if (!operator.isConnected()) return;

        // get.isPressed(get.Ingest(), () -> _intake.setState(IntakeState.Forward));
        // get.isPressed(get.Outgest(), () -> _intake.setState(IntakeState.Backward));
        // get.isPressed(get.IngestSlow(), () -> _intake.setState(IntakeState.ForwardSlow));
        // get.isNotPressed(List.of(get.Outgest(), get.Ingest(), get.IngestSlow()), () -> _intake.setState(IntakeState.Idle));

        // get.isPressed(get.Shoot(), () -> {
        //     _shooter.fire();
        //     _kicker.setToIntendedState();
        //     _spindexer.setState(SpindexerState.Forward);
        // });
        // get.isNotPressed(get.Shoot(), () -> {
        //     _spindexer.setToIntendedState();
        //     _kicker.setState(KickerState.Idle);
        //     _shooter.setState(ShooterState.Idle);
        // });

        // get.isPressed(get.Pass(), () -> {
        //     _shooter.setState(ShooterState.Passing);
        //     _kicker.setToIntendedState();
        //     _spindexer.setState(SpindexerState.Forward);
        // });

        // get.isNotPressed(get.Pass(), () -> {
        //     _shooter.setState(ShooterState.Idle);
        //     // _kicker.setState(KickerState.Idle);
        //     _spindexer.setToIntendedState();
        // });

        // get.isPressed(get.FreeFire(), ()-> _kicker.setIntendedState(KickerState.Firing));
        // get.isPressed(get.HoldMode(), ()-> _kicker.setIntendedState(KickerState.HoldAndFire));

        // get.isPressed(get.AutoTargeting(), () -> _shooter.setIntendedState(ShooterState.Targeting));
        // get.isPressed(get.OverrideLongShot(), () -> _shooter.setIntendedState(ShooterState.Far));
        // get.isPressed(get.OverrideMidShot(), () -> _shooter.setIntendedState(ShooterState.Mid));
        // get.isPressed(get.OverrideShortShot(), () -> _shooter.setIntendedState(ShooterState.Close));

        // get.isPressed(get.ToggleSpindexer(), () -> _spindexer.toggleSpindexer());
        // get.isPressed(get.IntakePivotToggle(), () -> _intakePivot.togglePos());
        // get.isPressed(get.IntakePivotRehome(), () -> _intakePivot.setState(IntakePivotState.Rehoming));
    }

    public void Dashboard() {
        SmartDashboard.putBoolean("Driver Connected?", driver.isConnected());
        SmartDashboard.putBoolean("Operator Connected?", operator.isConnected());
    }

    // get rotation to hub
    private Rotation2d getHubTargetRotation() {
        Pose2d robotPose = Robot.getOdometryInstance().getPose();
        double hubX = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
                       FieldConstants.hubPosBlue.getX() : FieldConstants.hubPosRed.getX());
        double hubY = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
                       FieldConstants.hubPosBlue.getY() : FieldConstants.hubPosRed.getY());

        double dx = hubX - robotPose.getX();
        double dy = hubY - robotPose.getY();
        Rotation2d hubAngle = new Rotation2d(Math.atan2(dy, dx));
        //intake is front of the robot, shooter is 90 degree ofset
        return hubAngle.plus(Rotation2d.fromDegrees(90)); 
    }
}
