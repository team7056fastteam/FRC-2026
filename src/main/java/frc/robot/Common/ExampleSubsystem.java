package frc.robot.Common;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase{
    static ExampleSubsystem _example = null;
    public enum ExampleState {kIdle, kUnIdle}

    ExampleState state = ExampleState.kIdle;

    @Override
    public void periodic() {
       switch(state){
        case kIdle:
            break;
        case kUnIdle:
            break;
       }
    }

    public void setState(ExampleState state) {
        this.state = state;
    }

    public ExampleState getState() {
        return state;
    }

    public void stop() {
        state = ExampleState.kIdle;
    }

    public static ExampleSubsystem getInstance(){
        if(_example == null){
            _example = new ExampleSubsystem();
        }
        return _example;
    }

    public void dashboard() {
        SmartDashboard.putString("Example State", state.toString());
    }
}
