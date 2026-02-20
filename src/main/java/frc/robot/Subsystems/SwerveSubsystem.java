package frc.robot.Subsystems;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Common.SwerveHeadingController;
import frc.robot.Common.SwerveModule;
import frc.robot.Constants.*;

public class SwerveSubsystem extends SubsystemBase {

    private SwerveState state = SwerveState.Idle;
    private ChassisSpeeds inputSpeeds = new ChassisSpeeds(0, 0, 0);

    public enum SwerveState {
        Idle,
        Lock_Wheels,
        TeleOp,
        Path_Following,
        Heading_Control,
        Auto_Extra
    }

    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveMotorReversed,
        DriveConstants.kFrontLeftTurningMotorReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveMotorReversed,
        DriveConstants.kFrontRightTurningMotorReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveMotorReversed,
        DriveConstants.kBackLeftTurningMotorReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveMotorReversed,
        DriveConstants.kBackRightTurningMotorReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed
    );

    public SwerveSubsystem() {
        runChassis(new ChassisSpeeds(0, 0, 0));
    }

    public void runChassis(ChassisSpeeds speeds) {

        // kill tiny jitter speeds
        double minLinear = 0.05;
        double minAngular = 0.05;

        if (Math.abs(speeds.vxMetersPerSecond) < minLinear)
            speeds.vxMetersPerSecond = 0;

        if (Math.abs(speeds.vyMetersPerSecond) < minLinear)
            speeds.vyMetersPerSecond = 0;

        if (Math.abs(speeds.omegaRadiansPerSecond) < minAngular)
            speeds.omegaRadiansPerSecond = 0;

        SwerveModuleState[] moduleStates =
            DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // max speed capping 
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond
        );

        desiredStates[0] = optimize(desiredStates[0], frontLeft.getPosition().angle);
        desiredStates[1] = optimize(desiredStates[1], frontRight.getPosition().angle);
        desiredStates[2] = optimize(desiredStates[2], backLeft.getPosition().angle);
        desiredStates[3] = optimize(desiredStates[3], backRight.getPosition().angle);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    private SwerveModuleState optimize(SwerveModuleState desired, Rotation2d currentAngle) {
        double delta = desired.angle.minus(currentAngle).getRadians();
        delta = Math.atan2(Math.sin(delta), Math.cos(delta));

        if (Math.abs(delta) > Math.PI / 2) {
            return new SwerveModuleState(
                -desired.speedMetersPerSecond,
                desired.angle.plus(Rotation2d.fromRadians(Math.PI))
            );
        }

        return desired;
    }

    public void feedSwerveSpeeds(ChassisSpeeds speeds) {
        if (state == SwerveState.Path_Following) {
            inputSpeeds = speeds;
            return;
        }
        inputSpeeds = speeds;
    }

    public void drive(ChassisSpeeds speeds) {
        state = SwerveState.Path_Following;
        feedSwerveSpeeds(speeds);
    }

    public void stop() {
        inputSpeeds = new ChassisSpeeds(0, 0, 0);
        runChassis(inputSpeeds);
    }

    public void setState(SwerveState newState) {
        state = newState;
    }

    @Override
    public void periodic() {
        switch (state) {
            case Idle:
                runChassis(new ChassisSpeeds(0, 0, 0));
                break;
            case TeleOp:
            case Heading_Control:
            case Path_Following:
            case Auto_Extra:
                runChassis(inputSpeeds);
                break;
            case Lock_Wheels:
                frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
                frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
                backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
                backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
                break;
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

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }

    public void dashboard() {
        SmartDashboard.putString("Swerve State", state.toString());
        SmartDashboard.putString("FL", frontLeft.getState().toString());
        SmartDashboard.putString("FR", frontRight.getState().toString());
        SmartDashboard.putString("BL", backLeft.getState().toString());
        SmartDashboard.putString("BR", backRight.getState().toString());
    }
}
