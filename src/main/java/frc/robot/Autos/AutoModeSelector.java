package frc.robot.Autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Common.FastAutoBase;
import frc.robot.Common.PathLoader;

public class AutoModeSelector {

    private static FastAutoBase DoNothing = new DoNothing();

    private static ArrayList<FastAutoBase> LoadedAutos = new ArrayList<>();

    private static SendableChooser<FastAutoBase> autoChooser = new SendableChooser<>();

    public AutoModeSelector(){
        LoadedAutos.clear();
        LoadedAutos = PathLoader.getAutos();

        //Auto Chooser
        autoChooser.setDefaultOption(DoNothing.getName(), DoNothing);

        addAuto(DoNothing);

        if(LoadedAutos != null){
            for(FastAutoBase auto : LoadedAutos){
                addAuto(auto);
            }
        }

        SmartDashboard.putData("Auto choices", autoChooser);
    }

    public void resetAutos(){
        LoadedAutos.clear();
        LoadedAutos = PathLoader.getAutos();

        //Auto Chooser
        autoChooser.setDefaultOption(DoNothing.getName(), DoNothing);

        addAuto(DoNothing);
        
        if(LoadedAutos != null){
            for(FastAutoBase auto : LoadedAutos){
                addAuto(auto);
            }
        }

        SmartDashboard.putData("Auto choices", autoChooser);
    }

    public FastAutoBase getAutoMode(){
        if(autoChooser.getSelected() == null){
            return null;
        }
        SmartDashboard.putString("SelectedAuto", autoChooser.getSelected().getName());
        SmartDashboard.putString("StartingPos", autoChooser.getSelected().getStartingPose().toString());
        return autoChooser.getSelected();
    }

    void addAuto(FastAutoBase auto){
        autoChooser.addOption(auto.getName(), auto);
    }
}
