package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Common.FeedForwardValues;
import frc.robot.Common.PIDValues;

public class IntakePivot extends SubsystemBase {
    public enum IntakePivotState{Idle, Open, Rehoming}
    IntakePivotState state = IntakePivotState.Idle;
    SparkMax intakePivotMotor;
    SparkMaxConfig motorConfig;
    TrapezoidProfile mProfile;
    TrapezoidProfile.State goal = new TrapezoidProfile.State();
    TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    Debouncer homeDebouncer;
    boolean homingFlag = false;
    private double ks;
    private double kg;
    private double kv;
    private double ka;

    private double kP;
    private double kI;
    private double kD;

    public IntakePivot(){
        intakePivotMotor = new SparkMax(IntakePivotConstants.IntakePivotMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.inverted(IntakePivotConstants.ReversedIntakePivot)
            .idleMode(IdleMode.kBrake);
        setGains(IntakePivotConstants.IntakePivotFF);
        setPids(IntakePivotConstants.IntakePivotPID);
        motorConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);
        motorConfig.closedLoop.feedForward
            .kG(kg)
            .kA(ka)
            .kS(ks)
            .kV(kv);
        mProfile = new TrapezoidProfile(new Constraints(IntakePivotConstants.IntakePivotMaxVelocity, IntakePivotConstants.IntakePivotMaxAcceleration));
        goal = new TrapezoidProfile.State(0.1, 0);
        setpoint = new TrapezoidProfile.State(intakePivotMotor.getEncoder().getPosition(), 0);

        intakePivotMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
        homeDebouncer = new Debouncer(IntakePivotConstants.HomeDebouncerTime);
    }

    @Override
    public void periodic() {
        switch(state){
            case Idle:
                intakePivotMotor.set(0);
                break;
            case Rehoming:
                intakePivotMotor.set(IntakePivotConstants.IntakePivotHomingVelocity);

                if(homeDebouncer.calculate(Math.abs(getVelocity()) < IntakePivotConstants.HomeVelocityThreshold)){
                    resetPosition();
                    setpoint = new TrapezoidProfile.State(intakePivotMotor.getEncoder().getPosition(), 0);
                    setState(IntakePivotState.Open);
                    setIntakePivotDown();
                    homingFlag = false;
                }
                break;
            case Open:
                setpoint = mProfile.calculate(0.02, setpoint, goal);
                intakePivotMotor.getClosedLoopController().setSetpoint(setpoint.position, ControlType.kPosition);
                if(homingFlag){
                    state = IntakePivotState.Rehoming;
                }
        }
    }

    public void stop() {
        intakePivotMotor.stopMotor();
    }

    public double getVelocity(){
        return intakePivotMotor.getEncoder().getVelocity();
    }

    public double getPosition(){
        return intakePivotMotor.getEncoder().getPosition();
    }

    public void setPosition(double position) {
        goal =
          new TrapezoidProfile.State(
              MathUtil.clamp(position, IntakePivotConstants.MinPosition, IntakePivotConstants.MaxPosition), 0);
    }

    public void dashboard() {
        SmartDashboard.putNumber("Intake Pivot Pos", intakePivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Pivot Current", intakePivotMotor.getOutputCurrent());
        SmartDashboard.putString("Intake Pivot State", state.toString());
        SmartDashboard.putNumber("Intake Pivot RPM", intakePivotMotor.getEncoder().getVelocity());
    }

    public void setState(IntakePivotState state){
        this.state = state;
    }

    public void setIntakePivotUp(){
        setPosition(Units.degreesToRotations(0));
    }

    public void setIntakePivotDown(){
        setPosition(Units.degreesToRotations(90));
    }

    public void togglePos(){
        if(state == IntakePivotState.Open){
            if(isNear(goal.position, IntakePivotConstants.DownGoalState, 1e-4)){
                setIntakePivotUp();
            } else if(isNear(goal.position, IntakePivotConstants.UpGoalState, 1e-4)){
                setIntakePivotDown();
            }
        }
    }

    public boolean inPos() {
            return state == IntakePivotState.Open && Math.abs(
                intakePivotMotor.getEncoder().getPosition() - goal.position
            ) < Units.degreesToRotations(5);
    }

    public void resetPosition(){
        intakePivotMotor.getEncoder().setPosition(Units.degreesToRotations(90));
    }

    public void rehome(){
        homingFlag = true;
    }

    //required due to java floating-point precision 
    private boolean isNear(double a, double b, double tolerance) {
        return Math.abs(a - b) < tolerance;
    }

    public void setGains(FeedForwardValues feed){
        this.ks = feed.getKS();
        this.kg = feed.getKG();
        this.kv = feed.getKV();
        this.ka = feed.getKA();
    }

    public void setPids(PIDValues pids) {
        this.kP = pids.getP();
        this.kI = pids.getI();
        this.kD = pids.getD();
    }

    public static final class IntakePivotConstants{
        //TODO find constants
        public static final int IntakePivotMotorID = 9;
        public static final boolean ReversedIntakePivot = false;
        public static final double IntakePivotHomingVelocity = -.1;
        public static final double IntakePivotMaxVelocity = 50;
        public static final double IntakePivotMaxAcceleration = 90;
        public static final double HomeDebouncerTime = .25;
        public static final double HomeVelocityThreshold = 0.01;
        public static final PIDValues IntakePivotPID = new PIDValues(1.2, 0, 0.05);
        public static final FeedForwardValues IntakePivotFF = new FeedForwardValues(0.05, 0.20, 0,0);
        public static final double MinPosition = Units.degreesToRotations(0);
        public static final double MaxPosition = Units.degreesToRotations(90);
        public static final double DownGoalState = Units.degreesToRotations(90);
        public static final double UpGoalState = Units.degreesToRotations(0);
    }
    
}
