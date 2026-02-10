package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Common.FastSubsystemBase;

public class Intake extends SubsystemBase {
    public enum IntakeState{Idle,Forward, ForwardSlow, Backward}
    IntakeState state = IntakeState.Idle;
    SparkMax intakeMotor;
    SparkMaxConfig motorConfig;

    public Intake(){
        intakeMotor = new SparkMax(IntakeConstants.IntakeMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.inverted(IntakeConstants.ReversedIntake);
        intakeMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }


    @Override
    public void periodic() {
        switch (state) {
            case Forward:
                intakeMotor.set(IntakeConstants.IntakeForwardVelocity);
                break;
            case Idle:
                intakeMotor.set(0);
                break;   
            case ForwardSlow:
                intakeMotor.set(IntakeConstants.IntakeForwardSlowVelocity);
            case Backward:
                intakeMotor.set(IntakeConstants.IntakeBackwardVelocity);
        }
    }

    // @Override
    // public void stop() {
    //     intakeMotor.stopMotor();
    // }

    // @Override
    public void dashboard() {
        SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
        SmartDashboard.putString("Intake State", state.toString());
    }

    public void setState(IntakeState state){
        this.state = state;
    }
    
    
    public static final class IntakeConstants{
        //TODO Find Actual Constants
        public static final int IntakeMotorID = 10;
        public static final boolean ReversedIntake = false;
        public static final double IntakeForwardVelocity = 0;
        public static final double IntakeForwardSlowVelocity = 0;
        public static final double IntakeBackwardVelocity = -.1;
    }

}
