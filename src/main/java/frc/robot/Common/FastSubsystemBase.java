package frc.robot.Common;

import frc.robot.KurtLogger;

public abstract class FastSubsystemBase {
    FastSubsystemBase subsystem = null;

    public abstract void Init(KurtLogger logger);
    public abstract void run();
    public abstract void stop();
    public abstract void dashboard();
}