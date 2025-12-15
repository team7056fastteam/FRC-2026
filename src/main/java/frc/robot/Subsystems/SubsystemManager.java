package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;

import frc.robot.KurtLogger;
import frc.robot.Common.FastSubsystemBase;

public class SubsystemManager {
    public static SubsystemManager mInstance = null;

    private List<FastSubsystemBase> subsystems = new ArrayList<>();

    private SwerveSubsystem _swerve;

    KurtLogger logger;
    
    public void registerSubsystems(){
        //Create the subsystems
        _swerve = new SwerveSubsystem();
        // _outTake = new BackUpIntake();
        // Initialize and add subsystems to the list
        subsystems.add(_swerve);
        // subsystems.add(_outTake);
    }

    public static SubsystemManager getInstance() {
        if (mInstance == null) {
            mInstance = new SubsystemManager();
        }
        return mInstance;
    }

    public void setLogger(KurtLogger logger){
        this.logger = logger;
    }

    public void init(){
        for (FastSubsystemBase subsystem : subsystems) {
            subsystem.Init(logger);
        }
    }

    public void run(){
        for (FastSubsystemBase subsystem : subsystems) {
            subsystem.run();
        }
    }

    public void stop(){
        logger = null;
        for (FastSubsystemBase subsystem : subsystems) {
            subsystem.stop();
        }
    }

    public void dashboard(){
        for (FastSubsystemBase subsystem : subsystems) {
            subsystem.dashboard();
        }
    }
}
