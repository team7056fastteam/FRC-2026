package frc.robot.AutoCommands;

import frc.robot.Robot;
import frc.robot.Common.FastCommand;
import frc.robot.Common.Point;

public class PassXYCommand extends FastCommand{
    double targetx, targety, targetError;

    public PassXYCommand(Point point, double error){
        targetx = point.getX();
        targety = point.getY();
        targetError = error;
    }

    @Override
    public void init() {}

    @Override
    public void run() {}

    @Override
    public Boolean isFinished() {
        return Math.abs(targetx - Robot.getPose().getX()) < targetError && Math.abs(targety - Robot.getPose().getY()) < targetError;
    }

    @Override
    public void end() {}
    
}
