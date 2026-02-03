package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Common.FastCommand;
import frc.robot.Subsystems.Kicker;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Spindexer;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.Kicker.KickerState;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Subsystems.Spindexer.SpindexerState;

public class Shoot extends FastCommand {
    private double wait = 0;
    private SubsystemManager _manager;
    private Spindexer _spindexer;
    private Kicker _kicker;
    private Shooter _shooter;
    private Timer timer;

    public Shoot(double time){
        wait = time;
        timer = new Timer();
    }

    @Override
    public void init() {
        _manager = SubsystemManager.getInstance();
        _spindexer = _manager.getSpindexerInstance();
        _kicker = _manager.getKickerInstance();
        _shooter = _manager.getShooterInstance();

        timer.reset();
        timer.start();

        _spindexer.setState(SpindexerState.Forward);
        _kicker.setState(KickerState.Firing);
        _shooter.setState(ShooterState.Auto);
    }

    @Override
    public void run() {}

    @Override
    public Boolean isFinished() {
        return timer.get() > wait;
    }

    @Override
    public void end() {
        _kicker.setState(KickerState.Idle);
        _spindexer.setToIntendedState();
        _shooter.setState(ShooterState.Idle);
    }
    
}
