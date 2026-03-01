package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
                intakeMotor.set(1);
                break;
            case Idle:
                intakeMotor.set(0);
                break;   
            case ForwardSlow:
                intakeMotor.set(0.5);
                break;
            case Backward:
                intakeMotor.set(-0.5);
                break;
        }
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    public void dashboard() {
        SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
        SmartDashboard.putString("Intake State", state.toString());
        SmartDashboard.putNumber("Intake RPM", intakeMotor.getEncoder().getVelocity());
    }

    public void setState(IntakeState state){
        this.state = state;
    }
    
    public static final class IntakeConstants{
        //TODO find constants
        public static final int IntakeMotorID = 10;
        public static final boolean ReversedIntake = false;
        public static final double IntakeForwardSpeed = 1.0;
        public static final double IntakeForwardSlowSpeed = 0.5;
        public static final double IntakeBackwardSpeed = -0.5;
    }

}
