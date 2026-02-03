package frc.robot.Subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Common.FastSubsystemBase;
import frc.robot.KurtLogger;

public class Spindexer extends FastSubsystemBase {

    SparkMax spindexerMotor;
    SparkMaxConfig motorConfig;
    KurtLogger logger;
    private Timer timer = new Timer();

    public enum SpindexerState{Idle, Forward, ForwardSlow}
    SpindexerState state = SpindexerState.Idle;
    SpindexerState lastState;
    SpindexerState intendedState = SpindexerState.Idle;
    public Spindexer(){
        spindexerMotor = new SparkMax(SpindexerConstants.SpindexerMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.inverted(SpindexerConstants.ReversedSpindexer);
        spindexerMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }

    @Override
    public void Init(KurtLogger logger) {
        this.logger = logger;
    }

    @Override
    public void run() {
        switch(state){
            case Forward:
            spindexerMotor.set(SpindexerConstants.SpindexerForwardVelocity);
                break;
            case ForwardSlow:
            spindexerMotor.set(SpindexerConstants.SpindexerForwardSlowVelocity);
                break;
            case Idle:
            spindexerMotor.set(0);
                break;
        }
    }

    @Override
    public void stop() {
       logger = null;
       spindexerMotor.stopMotor();
    }

    @Override
    public void dashboard() {
        SmartDashboard.putNumber("Spindexer Current", spindexerMotor.getOutputCurrent());
        SmartDashboard.putString("Spindexer State", state.toString());
    }

    public void setState(SpindexerState state){
        this.state = state;
    }

    public void toggleSpindexer(){
        timer.start();
        if(timer.get() > .2){
          if(intendedState == SpindexerState.ForwardSlow){
                intendedState = SpindexerState.Idle;
            } else if(intendedState == SpindexerState.Idle){
                intendedState = SpindexerState.ForwardSlow;
            } else{
                intendedState = SpindexerState.Idle;
        }
        timer.reset();
     }
    }

    public void setToLastState(){
        state = lastState;
    }

    public void setToIntendedState(){
        state = intendedState;
    }
    
    public static final class SpindexerConstants{
        // TODO Find Actual Constants
        public static final int SpindexerMotorID = 0;
        public static final boolean ReversedSpindexer = false;

        public static final double SpindexerForwardVelocity = 0;
        public static final double SpindexerForwardSlowVelocity = 0;
    }

}
