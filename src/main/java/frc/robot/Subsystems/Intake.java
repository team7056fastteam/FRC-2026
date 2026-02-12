package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Common.FeedForwardValues;
import frc.robot.Common.PIDValues;

public class Intake extends SubsystemBase {
    public enum IntakeState{Idle,Forward, ForwardSlow, Backward}
    IntakeState state = IntakeState.Idle;
    SparkMax intakeMotor;
    SparkMaxConfig motorConfig;
    private double ks;
    private double kg;
    private double kv;
    private double ka;

    private double kP;
    private double kI;
    private double kD;


    public Intake(){
        intakeMotor = new SparkMax(IntakeConstants.IntakeMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.inverted(IntakeConstants.ReversedIntake)
        .idleMode(IdleMode.kBrake);
        setGains(IntakeConstants.IntakeFF);
        setPids(IntakeConstants.IntakePID);
        motorConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);
        motorConfig.closedLoop.feedForward
            .kS(ks)
            .kG(kg)
            .kV(kv)
            .kA(ka);
        intakeMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }


    @Override
    public void periodic() {
        switch (state) {
            case Forward:
                intakeMotor.getClosedLoopController().setSetpoint(IntakeConstants.IntakeForwardVelocity, ControlType.kVelocity);
                break;
            case Idle:
                intakeMotor.set(0);
                break;   
            case ForwardSlow:
                intakeMotor.getClosedLoopController().setSetpoint(IntakeConstants.IntakeForwardSlowVelocity, ControlType.kVelocity);
                break;
            case Backward:
                intakeMotor.getClosedLoopController().setSetpoint(IntakeConstants.IntakeBackwardVelocity, ControlType.kVelocity);
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
    
    
    public static final class IntakeConstants{
        //TODO Find Actual Constants
        public static final int IntakeMotorID = 10;
        public static final boolean ReversedIntake = false;
        public static final double IntakeForwardVelocity = 4000;
        public static final double IntakeForwardSlowVelocity = 2000;
        public static final double IntakeBackwardVelocity = -2000;
        public static final PIDValues IntakePID = new PIDValues(0.00015, 0, 0);
        public static final FeedForwardValues IntakeFF = new FeedForwardValues(0.02,0,(1.0 / 5676.0),0);
    }

}
