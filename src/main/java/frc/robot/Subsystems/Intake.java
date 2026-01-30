package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.KurtLogger;
import frc.robot.Common.FastSubsystemBase;

public class Intake extends FastSubsystemBase {
    //TODO Real enum
    public enum IntakeState{Idle,Forward}
    IntakeState state = IntakeState.Idle;
    KurtLogger logger;
    SparkMax intakeMotor;
    SparkMaxConfig motorConfig;

    public Intake(){
        intakeMotor = new SparkMax(IntakeConstants.IntakeMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.inverted(IntakeConstants.ReversedIntake);
        intakeMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }

    @Override
    public void Init(KurtLogger logger) {
        this.logger = logger;
    }

    @Override
    public void run() {
        switch (state) {
            case Forward:
                intakeMotor.set(IntakeConstants.IntakeForwardVelocity);
                break;
            case Idle:
                intakeMotor.set(0);
                break;    
        }
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public void dashboard() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'dashboard'");
    }
    
    public static final class IntakeConstants{
        //TODO Find Actual Constants
        public static final int IntakeMotorID = 0;
        public static final boolean ReversedIntake = false;
        public static final double IntakeForwardVelocity = 0;
    }

}
