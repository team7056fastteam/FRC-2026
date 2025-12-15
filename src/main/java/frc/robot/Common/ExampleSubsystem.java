package frc.robot.Common;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.KurtLogger;
import frc.robot.KurtLogger.logType;

public class ExampleSubsystem extends FastSubsystemBase{
    static ExampleSubsystem _example = null;
    private KurtLogger logger;
    public enum ExampleState {kIdle, kUnIdle}

    ExampleState state = ExampleState.kIdle;
    
    @Override
    public void Init(KurtLogger logger) {
        this.logger = logger;
        state = ExampleState.kIdle;
    }

    @Override
    public void run() {
       switch(state){
        case kIdle:
            break;
        case kUnIdle:
            break;
       }
    }

    public void setState(ExampleState state) {
        this.state = state;
        if(logger != null){
            logger.logData(logType.event, state.toString(), "ExampleSubsystem");
        }
    }

    public ExampleState getState() {
        return state;
    }

    @Override
    public void stop() {
        logger = null;
        state = ExampleState.kIdle;
    }

    public static ExampleSubsystem getInstance(){
        if(_example == null){
            _example = new ExampleSubsystem();
        }
        return _example;
    }

    @Override
    public void dashboard() {
        SmartDashboard.putString("Example State", state.toString());
    }
}
