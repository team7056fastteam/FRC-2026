package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.KurtLogger;
import frc.robot.Common.FastSubsystemBase;
import frc.robot.Constants.FieldConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends FastSubsystemBase{
    public enum ShooterState{Idle, Targeting, Far, Close, Mid, Auto}
    ShooterState intendedState = ShooterState.Targeting;
    ShooterState state = ShooterState.Idle;
    KurtLogger logger;
    SparkFlex shooterMotor;
    SparkFlexConfig motorConfig;
    Pose2d currentPose;
    public Shooter(){
        shooterMotor = new SparkFlex(ShooterConstants.ShooterMotorID, MotorType.kBrushless);
        motorConfig = new SparkFlexConfig();
        motorConfig.inverted(ShooterConstants.ReversedShooter);
        motorConfig.closedLoop
            .p(ShooterConstants.kP)
            .i(ShooterConstants.kI)
            .d(ShooterConstants.kD)
            .outputRange(0, 1);
        motorConfig.closedLoop.feedForward.kV(ShooterConstants.kFF);
        shooterMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }

    @Override
    public void Init(KurtLogger logger) {
        this.logger = logger;
    }

    @Override
    public void run() {
        switch (state) {
            case Idle:
                shooterMotor.set(0);
                break;
            case Targeting:
                shooterMotor.getClosedLoopController().setSetpoint(calculateRPM(), ControlType.kVelocity);
                break;
            case Close:
                shooterMotor.getClosedLoopController().setSetpoint(2850, ControlType.kVelocity);
                break;
            case Mid:
                shooterMotor.getClosedLoopController().setSetpoint(3475, ControlType.kVelocity);
                break;
            case Far:
                shooterMotor.getClosedLoopController().setSetpoint(3900, ControlType.kVelocity);
                break;
            case Auto:
                shooterMotor.getClosedLoopController().setSetpoint(0, ControlType.kVelocity);
        }
    }

    @Override
    public void stop() {
        shooterMotor.stopMotor();
    }

    @Override
    public void dashboard() {
        SmartDashboard.putNumber("Shooter RPM", shooterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Current", shooterMotor.getOutputCurrent());
        SmartDashboard.putString("Shooter State", state.toString());
    }
    
public double calculateRPM() {
    if (currentPose == null) return 0.0;
    double horizontalDistance = currentPose.getTranslation().getDistance(FieldConstants.hubPos);
    double dz = FieldConstants.hubHeight - ShooterConstants.ShooterExitHeight;
    double denominator = 2 * Math.pow(Math.cos(ShooterConstants.ShooterExitAngle), 2) * (horizontalDistance * Math.tan(ShooterConstants.ShooterExitAngle) - dz);
    if (denominator <= 0 || horizontalDistance <= 0) {
        return 0.0;
    }
    //Linear velocity (m/s)
    double velocity = Math.sqrt(9.81 * horizontalDistance * horizontalDistance / denominator);
    double rpm = (velocity / (2 * Math.PI * ShooterConstants.ShooterWheelRadius)) * 60; //toRPM
    rpm = MathUtil.clamp(rpm, 0, 6000);
    return rpm;
}


    public void poseIn(Pose2d currentPose){
        this.currentPose = currentPose;
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

    public static final class ShooterConstants{
        //TODO Find Actual Constants
        public static final int ShooterMotorID = 0;
        public static final boolean ReversedShooter = false;
        public static final double kP = .0005;
        public static final double kI = 0;
        public static final double kD = .0001;
        public static final double kFF = 1.0/5676.0;
        public static final double ShooterExitHeight = Units.inchesToMeters(14.95);
        public static final double ShooterExitAngle = Math.toRadians(90); //degrees not correct
        public static final double ShooterWheelRadius = Units.inchesToMeters(2);
    }
}