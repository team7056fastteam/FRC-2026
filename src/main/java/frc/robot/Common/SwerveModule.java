package frc.robot.Common;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig turningMotorConfig = new SparkMaxConfig();

    private final RelativeEncoder driveEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.configure(driveMotorConfig.inverted(driveMotorReversed).apply(new EncoderConfig().velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec).positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)),
            com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
        turningMotor.configure(turningMotorConfig.inverted(turningMotorReversed),
            com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);

        driveEncoder = driveMotor.getEncoder();

        turningPidController = new PIDController(0.3, 0.2, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        driveEncoder.setPosition(0);
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = (absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        angle *= 2 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getCurrent() {
        return driveMotor.getOutputCurrent();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(),Rotation2d.fromRadians(getAbsoluteEncoderRad()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            driveMotor.set(0);
            turningMotor.set(0);
            return;
        }
        
        state.optimize(new Rotation2d(getAbsoluteEncoderRad()));

        double turnSpeed = turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());

        SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getDeviceID() + "] state", Math.toDegrees(getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("Encoder Error[" + absoluteEncoder.getDeviceID(), (state.angle.getRadians()-getAbsoluteEncoderRad()));

        turningMotor.set(turnSpeed);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }
    public void setDesiredStateUnrestricted(SwerveModuleState state) {
        
        state.optimize(new Rotation2d(getAbsoluteEncoderRad()));
        double turnSpeed = turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());

        turningMotor.set(turnSpeed);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }
}