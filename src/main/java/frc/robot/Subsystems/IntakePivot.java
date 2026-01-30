package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.KurtLogger;
import frc.robot.Common.FastSubsystemBase;

public class IntakePivot extends FastSubsystemBase {
    //TODO make real enum
    public enum IntakePivotState{Idle, Up, Down, StartingConfig, Rehoming}

    IntakePivotState state = IntakePivotState.Idle;

    SparkMax intakePivotMotor;
    SparkMaxConfig motorConfig;
    KurtLogger logger;

    public IntakePivot(){
        intakePivotMotor = new SparkMax(IntakePivotConstants.IntakePivotMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.inverted(IntakePivotConstants.ReversedIntakePivot);
        intakePivotMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }

    @Override
    public void Init(KurtLogger logger) {
        this.logger = logger;
    }

    @Override
    public void run() {
        switch(state){
            case Idle:

            break;

            case Up:

            break;

            case Down:

            break;

            case StartingConfig:

            break;

            case Rehoming:

            break;
        }
    }

    @Override
    public void stop() {
        intakePivotMotor.stopMotor();
    }

    @Override
    public void dashboard() {
        SmartDashboard.putNumber("Intake Pivot Pos", intakePivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Pivot Current", intakePivotMotor.getOutputCurrent());
    }

    public static final class IntakePivotConstants{
        //TODO Find Actual Constants
        public static final int IntakePivotMotorID = 0;
        public static final boolean ReversedIntakePivot = false;

    }
    
}
