package frc.robot.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Common.FastCommand;
import frc.robot.Common.SwerveHeadingController;
import frc.robot.Common.SwerveHeadingController.HeadingType;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.IntakePivot;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.IntakePivot.IntakePivotState;
import frc.robot.Subsystems.SwerveSubsystem.SwerveState;

public class MoveForwardAndIntake extends FastCommand{
    private double wait = 0;
    private SubsystemManager _manager;
    private SwerveSubsystem _swerve;
    private Intake _intake;
    private IntakePivot _intakePivot;
    private Timer timer;
    // private double holdingX = 0;
    private PIDController holdingXController;

    private SwerveHeadingController _mHeadingController;

    private final double kP = 0.07;
    private final double maxPower = 0.5;
    private final double forwardPower = 0.8;
    private double invertDrive = 1;
    private double targetX = 86.5;

    private double targetRadian = Math.PI;

    public MoveForwardAndIntake(double time){
        wait = time;
        timer = new Timer();
        holdingXController = new PIDController(kP, 0, 0);

        _mHeadingController = new SwerveHeadingController();
    }

    @Override
    public void init() {
        _manager = SubsystemManager.getInstance();
        _swerve = _manager.getSwerveInstance();
        _intake = _manager.getIntakeInstance();
        _intakePivot = _manager.getIntakePivotInstance();
        _swerve.setState(SwerveState.Auto_Extra);
        _intakePivot.setState(IntakePivotState.Down);
        _intake.setState(IntakeState.Forward);
        timer.reset();
        timer.start();

        if(Robot.getPose().getY() > 350){
            invertDrive = -1;
            targetRadian = 0;
        }

        if(Robot.getPose().getX() > 158.5){
            targetX = 230.5;
        }

        if(Robot.getPose().getX() > 138.5 && Robot.getPose().getX() < 178.5){
            targetX = 158.5;
        }

        _mHeadingController.setState(HeadingType.SNAP);
        _mHeadingController.setTarget(targetRadian);
    }

    @Override
    public void run() {
        double yPowerOffset = holdingXController.calculate(Robot.getPose().getX(),targetX);
        double zPower = _mHeadingController.calculate(Robot.getPose().getRotation().getRadians());
        yPowerOffset = MathUtil.clamp(yPowerOffset, -maxPower, maxPower);
        _swerve.feedSwerveSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-forwardPower*invertDrive,-yPowerOffset,zPower), Robot.getGyroscopeRotation2d()));
    }

    @Override
    public Boolean isFinished() {
        return timer.get() > wait;
    }

    @Override
    public void end() {
        _intake.setState(IntakeState.Idle);
        _intakePivot.setState(IntakePivotState.Up);
        _swerve.feedSwerveSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(), Robot.getGyroscopeRotation2d()));
    }
    
}
