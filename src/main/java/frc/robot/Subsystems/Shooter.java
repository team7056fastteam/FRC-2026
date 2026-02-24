package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Odometry;
import frc.robot.Robot;
import frc.robot.Common.PIDValues;
import frc.robot.Constants.FieldConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{
    public enum ShooterState{Idle, Targeting, Far, Close, Mid, Passing}
    ShooterState intendedState = ShooterState.Targeting;
    ShooterState state = ShooterState.Idle;
    SparkFlex shooterMotor;
    SparkFlexConfig motorConfig;
    Pose2d currentPose;
    double targetRPM;
    Odometry _odometry = Robot.getOdometryInstance();

    private double kP;
    private double kI;
    private double kD;

    public Shooter(){
        shooterMotor = new SparkFlex(ShooterConstants.ShooterMotorID, MotorType.kBrushless);
        motorConfig = new SparkFlexConfig();
        motorConfig.inverted(ShooterConstants.ReversedShooter)
            .idleMode(IdleMode.kCoast);
        setPids(ShooterConstants.ShooterPID);
        motorConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(0, 1);
        motorConfig.closedLoop.feedForward
            .kV(ShooterConstants.ShooterFF);
        shooterMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        currentPose = _odometry.getPose();
        switch (state) {
            case Idle:
                targetRPM = 0;
                shooterMotor.set(0);
                break;
            case Targeting:
                targetRPM = calculateRPM();
                shooterMotor.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
                break;
            case Close:
                targetRPM = 2850;
                shooterMotor.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
                break;
            case Mid:
                targetRPM = 3475;
                shooterMotor.getClosedLoopController().setSetpoint(3475, ControlType.kVelocity);
                break;
            case Far:
                targetRPM = 3900;
                shooterMotor.getClosedLoopController().setSetpoint(3900, ControlType.kVelocity);
                break;
            case Passing:
                targetRPM = 0;
                shooterMotor.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
        }
    }

    public void stop() {
        shooterMotor.stopMotor();
    }

    public void dashboard() {
        SmartDashboard.putNumber("Shooter RPM", shooterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Current", shooterMotor.getOutputCurrent());
        SmartDashboard.putString("Shooter State", state.toString());
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
    }
    
    public double calculateRPM() {
    if (currentPose == null) return 0.0;
    Pose2d shooterPose = currentPose.transformBy(
        new Transform2d(ShooterConstants.ShooterPoseOffsetX, ShooterConstants.ShooterPoseOffsetY, new Rotation2d()));
    double horizontalDistance = 
        shooterPose
            .getTranslation()
            .getDistance(
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? FieldConstants.hubPosBlue : FieldConstants.hubPosRed);

    horizontalDistance = MathUtil.clamp(horizontalDistance, 0.5, 8.0);

    double dz = FieldConstants.hubHeight - ShooterConstants.ShooterExitHeight;

    double theta = ShooterConstants.ShooterExitAngle;

    double denominator =
        2 * Math.pow(Math.cos(theta), 2) *
        (horizontalDistance * Math.tan(theta) - dz);

    if (denominator <= 0) return 0.0;

    double velocity =
        Math.sqrt(9.81 * horizontalDistance * horizontalDistance / denominator);

    velocity *= ShooterConstants.VelocityMultiplier;

    double rpm =
        (velocity / (2 * Math.PI * ShooterConstants.ShooterWheelRadius)) * 60.0;

    return MathUtil.clamp(rpm, 0, 6000);
}

    public void setState(ShooterState state){
        this.state = state;
    }

    public void fire(){
        state = intendedState;
    }

    public void setIntendedState(ShooterState state){
        intendedState = state;
    }

    public double getOutputCurrent(){
        return shooterMotor.getOutputCurrent();
    }

    public boolean atSpeed() {
        double target = shooterMotor.getClosedLoopController().getSetpoint();
        double actual = shooterMotor.getEncoder().getVelocity();
        return Math.abs(target - actual) < 100; // RPM tolerance
    }

    public void setPids(PIDValues pids){
        this.kP = pids.getP();
        this.kI = pids.getI();
        this.kD = pids.getD();
    }

    public static final class ShooterConstants{
        //TODO find constants
        public static final int ShooterMotorID = 13;
        public static final boolean ReversedShooter = false;
        public static final double ShooterExitHeight = Units.inchesToMeters(15.8);
        public static final double ShooterExitAngle = Math.toRadians(77.3);
        public static final double ShooterWheelRadius = Units.inchesToMeters(2);
        public static final double VelocityMultiplier = 1.15;
        public static final PIDValues ShooterPID = new PIDValues(.0005, 0, .0001);
        public static final double ShooterFF = 1.0 / 5676.0;
        public static final double ShooterPoseOffsetX = 0;
        public static final double ShooterPoseOffsetY = 0;
    }
}