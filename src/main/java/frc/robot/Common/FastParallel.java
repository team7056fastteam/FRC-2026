package frc.robot.Common;

import java.util.ArrayList;
import java.util.List;

public class FastParallel extends FastCommand{

    public ArrayList<FastCommand> mCommands = new ArrayList<>();

    public FastParallel(List<FastCommand> commands){
        mCommands = new ArrayList<>(commands);
    }

    @Override
    public void init() {
        for (FastCommand command : mCommands){
            command.init();
        }
    }

    @Override
    public void run() {
        for (FastCommand command : mCommands){
            command.run();
        }
    }

    @Override
    public Boolean isFinished() {
        for (FastCommand command : mCommands){
            if(!command.isFinished()){
                return false;
            }
        }
        return true;
    }

    @Override
    public void end() {
        for (FastCommand command : mCommands){
            command.end();
        }
    }
    
}
