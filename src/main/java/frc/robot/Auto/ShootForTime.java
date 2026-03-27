package frc.robot.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Kicker;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Spindexer;

public class ShootForTime extends Command {

    private final Shooter _shooter;
    private final Spindexer _spindexer;
    private final Kicker _kicker;
    private final Intake _intake;
    private final Shooter.ShooterState shooterState;
    private final double durationSeconds;
    private final boolean spinup;
    private final boolean preload;
    private boolean intakeReady;

    private final Timer timer = new Timer();
    private final Timer intakeTimer = new Timer();

    public ShootForTime(Shooter.ShooterState shooterState, double durationSeconds, boolean spinup, boolean preload) {
        this._shooter = Robot.getShooterInstance();
        this._spindexer = Robot.getSpindexerInstance();
        this._kicker = Robot.getKickerInstance();
        this._intake = Robot.getIntakeInstance();
        this.shooterState = shooterState;
        this.durationSeconds = durationSeconds;
        this.spinup = spinup;
        this.preload = preload;

        addRequirements(_shooter, _spindexer, _kicker, _intake);
    }

    @Override
    public void initialize() {
        // Start shooter
        _shooter.setIntendedState(shooterState);
        _shooter.fire();

        timer.reset();
        timer.start();
        intakeTimer.reset();
        intakeTimer.start();
        intakeReady = false;
    }

    @Override
    public void execute() {
        // Only run spindexer and kicker if shooter is at speed
        if (_shooter.atSpeed() && !spinup) {
            intakeReady = true;
            _spindexer.setState(Spindexer.SpindexerState.Forward);
            _kicker.setState(Kicker.KickerState.Firing);
        }
        if(intakeReady){
            if(!preload){
                if(DriveConstants.IntakeBackAndForthEnabled){
                    if(timer.get() < DriveConstants.IntakeForwardTime){
                        _intake.setState(IntakeState.Forward);
                    } else if(timer.get() < DriveConstants.IntakeStopTime){
                        _intake.setState(IntakeState.Idle);
                    } else if(timer.get() < DriveConstants.IntakeBackwardTime){
                        _intake.setState(IntakeState.Backward);
                    }
                    else{
                        timer.reset();
                        // timer.start();
                    }
                }
            } else{
                _intake.setState(IntakeState.Preload);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        _shooter.setState(Shooter.ShooterState.Idle);
        _spindexer.setState(Spindexer.SpindexerState.Idle);
        _kicker.setState(Kicker.KickerState.Idle);
        _intake.setState(IntakeState.Idle);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }
}
