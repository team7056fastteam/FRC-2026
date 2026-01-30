package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;

import frc.robot.KurtLogger;
import frc.robot.Common.FastSubsystemBase;

public class SubsystemManager {
    public static SubsystemManager mInstance = null;

    private List<FastSubsystemBase> subsystems = new ArrayList<>();

    private SwerveSubsystem _swerve;
    private Spindexer _spindexer;
    private Shooter _shooter;
    private Intake _intake;
    private IntakePivot _intakePivot;
    KurtLogger logger;
    
    public void registerSubsystems(){
        //Create the subsystems
        _swerve = new SwerveSubsystem();
        _spindexer = new Spindexer();
        _shooter = new Shooter();
        _intake = new Intake();
        _intakePivot = new IntakePivot();
        // Initialize and add subsystems to the list
        subsystems.add(_swerve);
        subsystems.add(_spindexer);
        subsystems.add(_shooter);
        subsystems.add(_intake);
        subsystems.add(_intakePivot);
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
