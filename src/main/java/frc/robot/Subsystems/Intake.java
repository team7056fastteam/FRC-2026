package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

public class Intake extends SubsystemBase {
    public enum IntakeState{Idle,Forward, ForwardSlow, Backward}
    IntakeState state = IntakeState.Idle;
    SparkMax intakeMotor;
    SparkMaxConfig motorConfig;

    public Intake(){
        intakeMotor = new SparkMax(IntakeConstants.IntakeMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.inverted(IntakeConstants.ReversedIntake)
            .idleMode(IdleMode.kCoast);
        intakeMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }


    @Override
    public void periodic() {
        switch (state) {
            case Forward:
                ChassisSpeeds speeds = Robot.getOdometryInstance().getFieldRelativeSpeeds();
                double robotSpeed = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).getNorm();
                double speedRatio = robotSpeed / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
                speedRatio = MathUtil.clamp(Math.pow(speedRatio, 0.4), 0, 1);

                double intakeSpeed = IntakeConstants.IntakeForwardSpeed - 
                    (IntakeConstants.IntakeForwardSpeed - IntakeConstants.IntakeForwardMinSpeed) * speedRatio;
                
                intakeMotor.set(intakeSpeed);
                break;
            case Idle:
                intakeMotor.set(0);
                break;   
            case ForwardSlow:
                intakeMotor.set(IntakeConstants.IntakeForwardSlowSpeed);
                break;
            case Backward:
                intakeMotor.set(IntakeConstants.IntakeBackwardSpeed);
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
        public static final int IntakeMotorID = 10;
        public static final boolean ReversedIntake = false;
        public static final double IntakeForwardMinSpeed = 0.5;
        public static final double IntakeForwardSpeed = .8;
        public static final double IntakeForwardSlowSpeed = 0.5;
        public static final double IntakeBackwardSpeed = -IntakeForwardSlowSpeed;
    }

}
