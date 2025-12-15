package frc.robot.AutoCommands;

import frc.robot.Common.FastCommand;
import frc.robot.Subsystems.SubsystemManager;

public class StopCommand extends FastCommand{

    @Override
    public void init() {
        SubsystemManager.getInstance().stop();
    }

    @Override
    public void run() {}

    @Override
    public Boolean isFinished() {
        return true;
    }

    @Override
    public void end() {}
}
