package frc.robot.Common;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class GravityNeoControl {
    private double ks;
    private double kg;
    private double kv;
    private double ka;

    private double kP;
    private double kI;
    private double kD;

    private double kMaxPosition = 100;
    private double kMinPosition = 0;

    private final SparkBaseConfig motorConfig;

    SparkMax motor;
    SparkMax[] followerMotors;

    RelativeEncoder encoder;
    CANcoder externalEncoder;
    TrapezoidProfile mProfile;
    TrapezoidProfile.State goal = new TrapezoidProfile.State();
    TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public GravityNeoControl(int motorID, int encoderId, double gearRatio, Boolean reveresed, double kMinPosition, double kMaxPosition, double kMaxVelocity, double kMaxAcceleration, PIDValues pids, FeedForwardValues feed){ 
        motor = new SparkMax(motorID, MotorType.kBrushless);
        mProfile = new TrapezoidProfile(new Constraints(kMaxVelocity, kMaxAcceleration));

        encoder = motor.getEncoder();
        externalEncoder = new CANcoder(encoderId);

        //externalEncoder.setPosition(externalEncoder.getAbsolutePosition().getValue()); might solve problem

        encoder.setPosition(getEncoderRadians());

        goal = new TrapezoidProfile.State(0, 0);
        setpoint = new TrapezoidProfile.State(getEncoderRadians(), 0);

        this.kMaxPosition = kMaxPosition;
        this.kMinPosition = kMinPosition;

        setGains(feed);
        setPids(pids);

        motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.inverted(reveresed);
        motorConfig.smartCurrentLimit(0,0,0);
        motorConfig.apply(new ClosedLoopConfig().pidf(kP, kI, kD, 0));
        motorConfig.apply(new EncoderConfig().positionConversionFactor(gearRatio).velocityConversionFactor(gearRatio));


        motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public GravityNeoControl(int motorID, double gearRatio, Boolean reveresed, double kMinPosition, double kMaxPosition, double kMaxVelocity, double kMaxAcceleration, double kMaxPower, PIDValues pids, FeedForwardValues feed){ 
        motor = new SparkMax(motorID, MotorType.kBrushless);
        mProfile = new TrapezoidProfile(new Constraints(kMaxVelocity, kMaxAcceleration));

        encoder = motor.getEncoder();

        encoder.setPosition(0);

        goal = new TrapezoidProfile.State(0, 0);
        setpoint = new TrapezoidProfile.State(encoder.getPosition(), 0);

        this.kMaxPosition = kMaxPosition;
        this.kMinPosition = kMinPosition;

        setGains(feed);
        setPids(pids);

        motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.inverted(reveresed);
        // motorConfig.smartCurrentLimit(0,0,0);
        motorConfig.apply(new ClosedLoopConfig().pidf(kP, kI, kD, 0).outputRange(-kMaxPower, kMaxPower));
        motorConfig.apply(new EncoderConfig().positionConversionFactor(gearRatio).velocityConversionFactor(gearRatio));


        motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void init(){
        setpoint = new TrapezoidProfile.State(getPosition(), 0);
    }

    public void setGains(FeedForwardValues feed) {
        this.ks = feed.getKS();
        this.kg = feed.getKG();
        this.kv = feed.getKV();
        this.ka = feed.getKA();
    }

    public void setPids(PIDValues pids) {
        this.kP = pids.getP();
        this.kI = pids.getI();
        this.kD = pids.getD();
    }

    public double armFeedForwardCalculate(
        double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
        return ks * Math.signum(velocityRadPerSec)
          + kg * Math.cos(positionRadians)
          + kv * velocityRadPerSec
          + ka * accelRadPerSecSquared;
    }

    public void MotorPeriodic(){
        if(externalEncoder != null){
            encoder.setPosition(getEncoderRadians());
        }
        setpoint = mProfile.calculate(0.02, setpoint, goal);
        motor.getClosedLoopController().setReference(setpoint.position, ControlType.kPosition,ClosedLoopSlot.kSlot0,
        armFeedForwardCalculate(setpoint.position, setpoint.velocity, 0));
    }

    public void setPosition(double position) {
        goal =
          new TrapezoidProfile.State(
              MathUtil.clamp(position, kMinPosition, kMaxPosition), 0);
    }

    public void resetPosition(double pos){
        encoder.setPosition(pos);
        System.out.println(Math.toDegrees(pos));
        setpoint = new TrapezoidProfile.State(encoder.getPosition(), 0);
    }

    public double getAppliedOutput(){
        return motor.getAppliedOutput();
    }

    public double getLowerLimit(){
        return kMinPosition;
    }

    public double getUpperLimit(){
        return kMaxPosition;
    }

    public double getPosition(){
        return encoder.getPosition();
    }

    public double getVelocity(){
        return encoder.getVelocity();
    }

    public double getEncoderRadians(){
        return Math.toRadians((externalEncoder.getPosition().getValueAsDouble())*360);
    }

    public double getEncoderDegrees(){
        return (externalEncoder.getPosition().getValueAsDouble())*360;
    }

    public void set(double pow){
        motor.set(pow);
    }

    public void zeroPower(){
        motor.set(0);
    }

    public void stop(){
        motor.stopMotor();
    }
}