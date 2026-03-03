package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Kicker extends SubsystemBase {
    public enum KickerState{Idle, Firing, Unstuckalate}
    KickerState state = KickerState.Idle;
    SparkMax kickerMotor;
    SparkMaxConfig motorConfig;

    Shooter _shooter;

    public Kicker(){
        kickerMotor = new SparkMax(KickerConstants.KickerMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.inverted(KickerConstants.ReversedKicker)
            .idleMode(IdleMode.kBrake);
        kickerMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
        _shooter = Robot.getShooterInstance();
    }

    @Override
    public void periodic() {
        switch (state) {
            case Firing:
                kickerMotor.set(KickerConstants.KickerFiringVelocity);
                break;
            case Idle:
                kickerMotor.stopMotor();
                break;
            case Unstuckalate:
                kickerMotor.set(KickerConstants.KickerUnstuckalateVelocity);
        }
    }

    public void stop() {
        kickerMotor.stopMotor();
    }

    public void dashboard() {
        SmartDashboard.putNumber("Kicker Current", kickerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Kicker RPM", kickerMotor.getEncoder().getVelocity());
        SmartDashboard.putString("Kicker State", state.toString());
    }

    public void setState(KickerState state){
        this.state = state;
    }

    public static final class KickerConstants{
        public static final int KickerMotorID = 12;
        public static final boolean ReversedKicker = false;
        public static final double KickerFiringVelocity = 1.0;
        public static final int FuelSensorID = 0;
        public static final double KickerUnstuckalateVelocity = -0.2;
        public static final double FuelSensorRange = Units.inchesToMeters(2);
    }

}
