package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Common.FeedForwardValues;
import frc.robot.Common.PIDValues;

public class Kicker extends SubsystemBase {
    public enum KickerState{Idle, Firing, HoldAndFire}
    KickerState state = KickerState.Idle;
    KickerState intendedState = KickerState.HoldAndFire;
    CANrange fuelSensor;
    SparkMax kickerMotor;
    SparkMaxConfig motorConfig;
    private double ks;
    private double kg;
    private double kv;
    private double ka;

    private double kP;
    private double kI;
    private double kD;

    Shooter _shooter;

    public Kicker(){
        kickerMotor = new SparkMax(KickerConstants.KickerMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        fuelSensor = new CANrange(KickerConstants.FuelSensorID);
        motorConfig.inverted(KickerConstants.ReversedKicker)
            .idleMode(IdleMode.kBrake);
        setGains(KickerConstants.KickerFF);
        setPids(KickerConstants.KickerPID);
        motorConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);
        motorConfig.closedLoop.feedForward
            .kA(ka)
            .kG(kg)
            .kV(kv)
            .kS(ks);
        kickerMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
        _shooter = Robot.getShooterInstance();
    }

    @Override
    public void periodic() {
        switch (state) {
            case Firing:
                kickerMotor.getClosedLoopController().setSetpoint(KickerConstants.KickerFiringVelocity, ControlType.kVelocity);
                break;
            case Idle:
                kickerMotor.stopMotor();
                break;
            case HoldAndFire:
                if(fuelSensor.getDistance().getValueAsDouble() < KickerConstants.FuelSensorRange){
                    if(_shooter.atSpeed()){
                        kickerMotor.getClosedLoopController().setSetpoint(KickerConstants.KickerFiringVelocity, ControlType.kVelocity);
                    } else {
                        kickerMotor.stopMotor();
                    }
                    
                } else {
                    kickerMotor.getClosedLoopController().setSetpoint(KickerConstants.KickerHoldingVelocity, ControlType.kVelocity);
                }
                break;
        }
    }

    public void stop() {
        kickerMotor.stopMotor();
    }

    public void dashboard() {
        SmartDashboard.putNumber("Kicker Current", kickerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Kicker RPM", kickerMotor.getEncoder().getVelocity());
        SmartDashboard.putString("Kicker State", state.toString());
        SmartDashboard.putNumber("Fuel Sensor Distance", fuelSensor.getDistance().getValueAsDouble());
    }

    public void setState(KickerState state){
        this.state = state;
    }

    public void setIntendedState(KickerState state){
        intendedState = state;
    }

    public void setToIntendedState(){
        state = intendedState;
    }

    public void setGains(FeedForwardValues feed){
        this.ks = feed.getKS();
        this.kg = feed.getKG();
        this.kv = feed.getKV();
        this.ka = feed.getKA();
    }
    
    public void setPids(PIDValues pids){
        this.kP = pids.getP();
        this.kI = pids.getI();
        this.kD = pids.getD();
    }

    public static final class KickerConstants{
        //TODO contants
        public static final int KickerMotorID = 12;
        public static final boolean ReversedKicker = false;
        public static final double KickerFiringVelocity = 4500;
        public static final int FuelSensorID = 0;
        public static final double KickerHoldingVelocity = 800;
        public static final PIDValues KickerPID = new PIDValues(0.00015, 0, 0);
        public static final FeedForwardValues KickerFF = new FeedForwardValues(0.02, 0, (1.0 / 5676.0), 0);
        public static final double FuelSensorRange = Units.inchesToMeters(2);
    }

}
