package frc.robot.Common;

import java.util.ArrayList;
import java.util.List;

public class FastPetitionSeries extends FastPetition{

    ArrayList<FastPetition> petitions = new ArrayList<>();
    FastPetition petition;

    public FastPetitionSeries(List<FastPetition> petitions){
        this.petitions = new ArrayList<>(petitions);
    }

    @Override
    public void run() {
        if(petition == null){
            if(petitions.isEmpty()){
                return;
            }

            petition = petitions.remove(0);
        }

        petition.run();

        if(petition.isFinished()){
            petition = null;
        }
    }

    @Override
    public boolean isFinished() {
        return petitions.isEmpty() && petition == null;
    }

}
