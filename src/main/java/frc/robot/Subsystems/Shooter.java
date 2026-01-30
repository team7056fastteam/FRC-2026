package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.KurtLogger;
import frc.robot.Common.FastSubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends FastSubsystemBase{
    //TODO Shooter Enum
    public enum ShooterState{Idle, Targeting}
    ShooterState state = ShooterState.Idle;
    KurtLogger logger;
    SparkFlex shooterMotor;
    SparkFlexConfig motorConfig;

    public Shooter(){
        shooterMotor = new SparkFlex(ShooterConstants.ShooterMotorID, MotorType.kBrushless);
        motorConfig = new SparkFlexConfig();
        motorConfig.inverted(ShooterConstants.ReversedShooter);
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
                //TODO Real Targeting System
                break;
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
    }

    public static final class ShooterConstants{
        //TODO Find Actual Constants
        public static final int ShooterMotorID = 0;
        public static final boolean ReversedShooter = false;
    }
}