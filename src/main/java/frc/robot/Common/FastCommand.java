package frc.robot.Common;

public abstract class FastCommand {
    public abstract void init();
    
    public abstract void run();

    public abstract Boolean isFinished();

    public abstract void end();

}
