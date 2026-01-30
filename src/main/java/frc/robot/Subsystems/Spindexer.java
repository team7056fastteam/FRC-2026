package frc.robot.Subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Common.FastSubsystemBase;
import frc.robot.KurtLogger;

public class Spindexer extends FastSubsystemBase {

    SparkMax spindexerMotor;
    SparkMaxConfig motorConfig;

    KurtLogger logger;

    public enum SpindexerState{Idle, Forward, ForwardSlow}
    SpindexerState state = SpindexerState.Idle;

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
    }
    
    public static final class SpindexerConstants{
        // TODO Find Actual Constants
        public static final int SpindexerMotorID = 0;
        public static final boolean ReversedSpindexer = false;

        public static final double SpindexerForwardVelocity = 0;
        public static final double SpindexerForwardSlowVelocity = 0;
    }

}
