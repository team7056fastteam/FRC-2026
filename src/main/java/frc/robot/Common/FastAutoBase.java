package frc.robot.Common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public abstract class FastAutoBase {

    public abstract String getName();
    public abstract void routine() throws Exception;
    protected boolean m_active = false;

    public void run() {
        m_active = true;
        try {
            routine();
        } catch (Exception e) {
            System.out.println("Auto mode done, ended early");
            return;
        }
        System.out.println("Auto mode done");
    }
    public void stop() {
        m_active = false;
    }

    public boolean isActiveWithThrow() throws Exception {
        if (!m_active) {
            throw new Exception();
        }

        return m_active;
    }

    public void runCommand(FastCommand command) throws Exception{
        isActiveWithThrow();
        command.init();

        while(isActiveWithThrow() && !command.isFinished()){
            command.run();
            Timer.delay(0.02);
        }

        command.end();
    }

    public abstract Pose2d getStartingPose();
}
