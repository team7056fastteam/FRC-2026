package frc.robot.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Kicker;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Spindexer;

public class ShootForTime extends Command {

    private final Shooter _shooter;
    private final Spindexer _spindexer;
    private final Kicker _kicker;
    private final Shooter.ShooterState shooterState;
    private final double durationSeconds;

    private final Timer timer = new Timer();

    public ShootForTime(Shooter.ShooterState shooterState, double durationSeconds) {
        this._shooter = Robot.getShooterInstance();
        this._spindexer = Robot.getSpindexerInstance();
        this._kicker = Robot.getKickerInstance();
        this.shooterState = shooterState;
        this.durationSeconds = durationSeconds;

        addRequirements(_shooter, _spindexer, _kicker);
    }

    @Override
    public void initialize() {
        // Start shooter
        _shooter.setIntendedState(shooterState);
        _shooter.fire();

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Only run spindexer and kicker if shooter is at speed
        if (_shooter.atSpeed()) {
            _spindexer.setState(Spindexer.SpindexerState.Forward);
            _kicker.setState(Kicker.KickerState.Firing);
        }
    }

    @Override
    public void end(boolean interrupted) {
        _shooter.setState(Shooter.ShooterState.Idle);
        _spindexer.setToIntendedState(); // respects toggle behavior
        _kicker.setState(Kicker.KickerState.Idle);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }
}
