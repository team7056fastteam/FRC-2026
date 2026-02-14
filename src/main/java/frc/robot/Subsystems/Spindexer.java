package frc.robot.Subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Common.FeedForwardValues;
import frc.robot.Common.PIDValues;

public class Spindexer extends SubsystemBase {

    SparkMax spindexerMotor;
    SparkMaxConfig motorConfig;

    public enum SpindexerState{Idle, Forward, ForwardSlow}
    SpindexerState state = SpindexerState.Idle;
    SpindexerState intendedState = SpindexerState.Idle;

    private double ks;
    private double kg;
    private double kv;
    private double ka;

    private double kP;
    private double kI;
    private double kD;

    public Spindexer(){
        spindexerMotor = new SparkMax(SpindexerConstants.SpindexerMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.inverted(SpindexerConstants.ReversedSpindexer)
            .idleMode(IdleMode.kBrake);
        setGains(SpindexerConstants.SpindexerFF);
        setPids(SpindexerConstants.SpindexerPID);
        motorConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(0, 1);
        motorConfig.closedLoop.feedForward
            .kA(ka)
            .kG(kg)
            .kS(ks)
            .kV(kv);
        spindexerMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }


    @Override
    public void periodic() {
        switch(state){
            case Forward:
                spindexerMotor.getClosedLoopController().setSetpoint(SpindexerConstants.SpindexerForwardVelocity, ControlType.kVelocity);
                break;
            case ForwardSlow:
                spindexerMotor.getClosedLoopController().setSetpoint(SpindexerConstants.SpindexerForwardSlowVelocity, ControlType.kVelocity);
                break;
            case Idle:
                spindexerMotor.set(0);
                break;
        }
    }

    public void stop() {
       spindexerMotor.stopMotor();
    }

    public void dashboard() {
        SmartDashboard.putNumber("Spindexer Current", spindexerMotor.getOutputCurrent());
        SmartDashboard.putString("Spindexer State", state.toString());
    }

    public void setState(SpindexerState state){
        this.state = state;
    }

    public void toggleSpindexer(){
        if(intendedState == SpindexerState.ForwardSlow){
            intendedState = SpindexerState.Idle;
        } else {
            intendedState = SpindexerState.ForwardSlow;
        }
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
    
    public static final class SpindexerConstants{
        // TODO Find Actual Constants
        public static final int SpindexerMotorID = 11;
        public static final boolean ReversedSpindexer = false;

        public static final double SpindexerForwardVelocity = 800;
        public static final double SpindexerForwardSlowVelocity = 400;

        public static final PIDValues SpindexerPID = new PIDValues(0.0002, 0, 0);
        public static final FeedForwardValues SpindexerFF = new FeedForwardValues(0.02, 0, (1.0 / 5676.0), 0);
    }

}
