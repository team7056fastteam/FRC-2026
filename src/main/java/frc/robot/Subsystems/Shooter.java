package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Odometry;
import frc.robot.Robot;
import frc.robot.Common.PIDValues;
import frc.robot.Constants.FieldConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{
    public enum ShooterState{Idle, Targeting, Far, Close, Mid, Passing}
    ShooterState intendedState = ShooterState.Targeting;
    ShooterState state = ShooterState.Idle;
    SparkFlex shooterMotor;
    SparkFlexConfig motorConfig;
    Pose2d currentPose;
    double targetRPM;
    Odometry _odometry = Robot.getOdometryInstance();

    private double kP;
    private double kI;
    private double kD;

    public Shooter(){
        shooterMotor = new SparkFlex(ShooterConstants.ShooterMotorID, MotorType.kBrushless);
        motorConfig = new SparkFlexConfig();
        motorConfig.inverted(ShooterConstants.ReversedShooter)
            .idleMode(IdleMode.kCoast);
        setPids(ShooterConstants.ShooterPID);
        motorConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(0, 1);
        motorConfig.closedLoop.feedForward
            .kV(ShooterConstants.ShooterFF);
        shooterMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        currentPose = _odometry.getPose();
        switch (state) {
            case Idle:
                targetRPM = 0;
                shooterMotor.set(0);
                break;
            case Targeting:
                targetRPM = calculateRPM();
                shooterMotor.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
                break;
            case Close:
                targetRPM = 2850;
                shooterMotor.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
                break;
            case Mid:
                targetRPM = 3475;
                shooterMotor.getClosedLoopController().setSetpoint(3475, ControlType.kVelocity);
                break;
            case Far:
                targetRPM = 3900;
                shooterMotor.getClosedLoopController().setSetpoint(3900, ControlType.kVelocity);
                break;
            case Passing:
                targetRPM = 0;
                shooterMotor.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
        }
    }

    public void stop() {
        shooterMotor.stopMotor();
    }

    public void dashboard() {
        SmartDashboard.putNumber("Shooter RPM", shooterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Current", shooterMotor.getOutputCurrent());
        SmartDashboard.putString("Shooter State", state.toString());
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
    }

    public double calculateRPM() {
        if (currentPose == null) return 0.0;

        Pose2d shooterPose = currentPose.transformBy(
            new Transform2d(
                ShooterConstants.ShooterPoseOffsetX,
                ShooterConstants.ShooterPoseOffsetY,
                new Rotation2d()
            )
        );

        Translation2d hubPos = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
                                    FieldConstants.hubPosBlue : FieldConstants.hubPosRed;

        Translation2d shooterTranslation = shooterPose.getTranslation();

        Translation2d toHub = hubPos.minus(shooterTranslation);

        double horizontalDistance = toHub.getNorm();

        ChassisSpeeds speeds = _odometry.getFieldRelativeSpeeds();
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;

        Translation2d velocity = new Translation2d(vx, vy);

        //estimated flight time (if undershooting increase, if overshooting decrease)
        double flightTime = (horizontalDistance * .1) + .3;

        double radialVelocity =
            (velocity.getX() * toHub.getX() +
            velocity.getY() * toHub.getY())
            / horizontalDistance;
        
        horizontalDistance += radialVelocity * flightTime;

        double distanceInches = MathUtil.clamp(Units.metersToInches(horizontalDistance), 20, 160);

        // Quadratic regression (fits real-world data)
        double rpm =
            0.1828 * distanceInches * distanceInches
            - 17.592 * distanceInches
            + 3243;

        return MathUtil.clamp(rpm, 0, 6000);
    }

    public void setState(ShooterState state){
        this.state = state;
    }

    public void fire(){
        state = intendedState;
    }

    public void setIntendedState(ShooterState shooterState){
        intendedState = shooterState;
    }

    public double getOutputCurrent(){
        return shooterMotor.getOutputCurrent();
    }

    public boolean atSpeed() {
        double target = shooterMotor.getClosedLoopController().getSetpoint();
        double actual = shooterMotor.getEncoder().getVelocity();
        return Math.abs(target - actual) < 100; // RPM tolerance
    }

    public void setPids(PIDValues pids){
        this.kP = pids.getP();
        this.kI = pids.getI();
        this.kD = pids.getD();
    }

    public static final class ShooterConstants{
        public static final int ShooterMotorID = 13;
        public static final boolean ReversedShooter = true;
        public static final PIDValues ShooterPID = new PIDValues(.0005, 0, .0001);
        public static final double ShooterFF = 1.0 / 5676.0;
        public static final double ShooterPoseOffsetX = Units.inchesToMeters(-9); //wpi is weird, x is forward positive, y is left positive
        public static final double ShooterPoseOffsetY = Units.inchesToMeters(7);
    }
}