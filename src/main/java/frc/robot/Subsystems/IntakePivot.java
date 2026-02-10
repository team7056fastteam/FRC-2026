package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Common.FastSubsystemBase;

public class IntakePivot extends SubsystemBase {
    public enum IntakePivotState{Starting, Up, Down, Rehoming}

    IntakePivotState state = IntakePivotState.Starting;
    SparkMax intakePivotMotor;
    SparkMaxConfig motorConfig;
    private Timer timer = new Timer();

    public IntakePivot(){
        intakePivotMotor = new SparkMax(IntakePivotConstants.IntakePivotMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.inverted(IntakePivotConstants.ReversedIntakePivot);
        intakePivotMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        switch(state){
            case Starting:
            intakePivotMotor.stopMotor();
            break;
            case Up:
            intakePivotMotor.getClosedLoopController().setSetpoint(0, ControlType.kPosition);
            break;
            case Down:
            intakePivotMotor.getClosedLoopController().setSetpoint(Units.degreesToRotations(90), ControlType.kPosition);
            break;
            case Rehoming:
            // Move slowly downward until current spike
            intakePivotMotor.set(IntakePivotConstants.IntakePivotHomingVelocity);
            // Check if current exceeds threshold
            double current = intakePivotMotor.getOutputCurrent();
            double currentThreshold = 5.0; // Amps, tune this to detect the hard stop
            if (current > currentThreshold) {
                intakePivotMotor.stopMotor();
                intakePivotMotor.getEncoder().setPosition(Units.degreesToRotations(90)); //reset encoder
                state = IntakePivotState.Down;
            }
            break;
        }
    }

    // @Override
    // public void stop() {
    //     intakePivotMotor.stopMotor();
    // }

    // @Override
    public void dashboard() {
        SmartDashboard.putNumber("Intake Pivot Pos", intakePivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Pivot Current", intakePivotMotor.getOutputCurrent());
        SmartDashboard.putString("Intake Pivot State", state.toString());
    }

    public void setState(IntakePivotState state){
        this.state = state;
    }

    public void togglePos(){
        timer.start();
        if(timer.get() > .2){
            if(state == IntakePivotState.Down){
                state = IntakePivotState.Up;
            } else if(state == IntakePivotState.Up){
                state = IntakePivotState.Down;
            } else{
                state = IntakePivotState.Up;
            }
            timer.reset();
        }
    }

    public static final class IntakePivotConstants{
        //TODO Find Actual Constants
        public static final int IntakePivotMotorID = 9;
        public static final boolean ReversedIntakePivot = false;
        public static final double IntakePivotHomingVelocity = -.2;

    }
    
}
