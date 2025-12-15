package frc.robot.Common;

import java.util.ArrayList;
import java.util.List;

public class FastSeries extends FastCommand{

    public final ArrayList<FastCommand> mCommands;
    private FastCommand command;

    public FastSeries(List<FastCommand> commands){
        mCommands = new ArrayList<>(commands);
        command = null;
    }

    @Override
    public void init() {}

    @Override
    public void run() {
        if(command == null){
            if(mCommands.isEmpty()){
                return;
            }

            command = mCommands.remove(0);
            command.init();
        }

        command.run();

        if(command.isFinished()){
            command.end();
            command = null;
        }
    }

    @Override
    public Boolean isFinished() {
        return mCommands.isEmpty() && command == null;
    }

    @Override
    public void end() {}
    
}
