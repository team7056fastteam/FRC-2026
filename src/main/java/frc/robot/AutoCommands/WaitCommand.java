package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Common.FastCommand;

public class WaitCommand extends FastCommand{
    Timer time = new Timer();
    double waitTime;

    public WaitCommand(double seconds){
        waitTime = seconds;
    }

    @Override
    public void init() {
        time.reset();
        time.start();
    }

    @Override
    public void run() {}

    @Override
    public Boolean isFinished() {
        return time.get() > waitTime;
    }

    @Override
    public void end() {}
    
}
