package frc.robot.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.AutoCommands.StopCommand;
import frc.robot.Common.FastAutoBase;

public class DoNothing extends FastAutoBase{

    @Override
    public String getName() {
        return "Do Nothing :)";
    }

    @Override
    public void routine() throws Exception {
        runCommand(new StopCommand());
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d();
    }
    
}
