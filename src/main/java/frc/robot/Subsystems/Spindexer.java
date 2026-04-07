package frc.robot.Subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Common.PIDValues;

public class Spindexer extends SubsystemBase {

    SparkMax spindexerMotor;
    SparkMaxConfig motorConfig;

    public enum SpindexerState{Idle, Forward, ForwardSlow}
    SpindexerState state = SpindexerState.Idle;
    SpindexerState intendedState = SpindexerState.Idle;

    private double kP;
    private double kI;
    private double kD;

    public Spindexer(){
        spindexerMotor = new SparkMax(SpindexerConstants.SpindexerMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.inverted(SpindexerConstants.ReversedSpindexer)
            .idleMode(IdleMode.kBrake);
        setPids(SpindexerConstants.SpindexerPID);
        motorConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);
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

    public void setPids(PIDValues pids){
        this.kP = pids.getP();
        this.kI = pids.getI();
        this.kD = pids.getD();
    }
    
    public static final class SpindexerConstants{
        public static final int SpindexerMotorID = 11;
        public static final boolean ReversedSpindexer = true;

        public static final double SpindexerForwardVelocity = 5000;
        public static final double SpindexerForwardSlowVelocity = 1000;

        public static final PIDValues SpindexerPID = new PIDValues(0.0002, 0, 0);
    }
}
