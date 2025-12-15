package frc.robot.Common;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.AutoCommands.*;

public class PathLoader {
    static ArrayList<FastAutoBase> autos = new ArrayList<>();
    static ArrayList<FastParallel> autoPaths = new ArrayList<>();
    static ArrayList<FastCommand> convertAutoPaths = new ArrayList<>();
    static ArrayList<Curve> curves = new ArrayList<>();
    static ArrayList<CMDObject> cmdObjects = new ArrayList<>();
    static File[] autoJsonFiles;
    static File folder = new File(Filesystem.getDeployDirectory().getPath()+"/Paths");

    static final double passXYError = 15;
    static final boolean DebugAuto = false;

    public static SaveData loadFromFile(String filePath) throws IOException {
        Gson gson = new Gson();
        Type saveDataType = new TypeToken<SaveData>() {}.getType();
        try (FileReader reader = new FileReader(filePath)) {
            return gson.fromJson(reader, saveDataType);
        }
    }

    public static ArrayList<FastAutoBase> getAutos(){
        autoJsonFiles = folder.listFiles((dir, name) -> name.toLowerCase().endsWith(".json"));

        for(File autoFile : autoJsonFiles){
            curves.clear();
            cmdObjects.clear();
            if(autoFile.getName().endsWith("networktables.json")){
                continue;
            }
            System.out.println("Loaded Auto with Name: " + autoFile.getName().substring(0,autoFile.getName().length()-5));
            try{

                SaveData loadedData = loadFromFile(autoFile.getAbsolutePath());
        
                // Extract curves and CMDObjects
                ArrayList<CurveSaveData> loadedCurves = loadedData.getCurves();
                ArrayList<CMDObjectSaveData> loadedCmdObjects = loadedData.getCmdObjects();
                
                for (CurveSaveData curveData : loadedCurves) {
                    Curve curve = new Curve(
                            curveData.getStartPoint(),
                            curveData.getEndPoint(),
                            curveData.getConstraintPoint0(),
                            curveData.getConstraintPoint1()
                    );
                    curves.add(curve);
                }
            
                for (CMDObjectSaveData cmdObjData : loadedCmdObjects) {
                    CMDObject cmdObj = new CMDObject(
                            cmdObjData.getCmds(),
                            cmdObjData.getPos(),
                            cmdObjData.getSpline()
                    );
                    cmdObjects.add(cmdObj);
                }
                loadedCurves.clear();
                loadedCmdObjects.clear();
            }
            catch(Exception e){
                System.out.println(e.toString());
            }

            autoPaths.clear();
            convertAutoPaths.clear();

            for(Curve curve : curves){
                autoPaths.add(new FastParallel(List.of(new RunPathCommand(curve.getArrayPoints()))));
            }
            //if there are cmdObjects at all
            if(cmdObjects != null){
                for(CMDObject cmd : cmdObjects){
                    //get location of cmdObject to run list
                    Point CMDLocation = curves.get(cmd.spline).getPoints()[(int)(cmd.pos*curves.get(cmd.spline).points.size())];

                    ArrayList<FastCommand> fcmds = new ArrayList<>();
                    //get cmd type and add to list
                    for(CMD command : cmd.cmds){
                        switch (command.selectedCMD) {
                            case Wait:
                                fcmds.add(new WaitCommand(command.value));
                                break;
                        }
                    }
                    //create the series command group and add the passxycmd fist
                    FastSeries series = new FastSeries(List.of(new PassXYCommand(CMDLocation, passXYError)));
                    //add rest of cmds to the series group
                    series.mCommands.addAll(fcmds);
                    //add the series group to the specified parallel group
                    autoPaths.get(cmd.spline).mCommands.add(series);
                }
            }
            else{
                System.out.println("cmds are null");
            }

            // for(FastParallel fast : autoPaths){
            //     System.out.println("AutoPath " + autoPaths.indexOf(fast));
            //     for(FastCommand command : fast.mCommands){
            //         System.out.println(command.toString());
            //         if(command instanceof FastSeries){
            //             FastSeries tempSeries = (FastSeries) command;
            //             for(FastCommand commandsInSeries : tempSeries.mCommands){
            //               System.out.println(commandsInSeries.toString());
            //             }
            //         }
            //     }
            // }

            ArrayList<FastParallel> combinedPaths = new ArrayList<>();
            int i = 0;

            while (i < autoPaths.size()) {
                // If the current path contains wait commands, add it directly to the combined paths list
                if (doesSplineContainCMDGroupsWithWaitCommands(i)) {
                    combinedPaths.add(autoPaths.get(i));
                    i++;
                } else {
                    // Collect all subsequent paths without wait commands
                    ArrayList<FastParallel> pathsToBeCombined = new ArrayList<>();
                    ArrayList<Curve> newCurves = new ArrayList<>();
            
                    int j = i;
                    while (j < autoPaths.size()) {
                        pathsToBeCombined.add(autoPaths.get(j));
                        newCurves.add(curves.get(j));
                        j++;
                        if(doesSplineContainCMDGroupsWithWaitCommands(j)){
                            pathsToBeCombined.add(autoPaths.get(j));
                            newCurves.add(curves.get(j));
                            j++;
                            break;
                        }
                    }
            
                    // Create a new combined path
                    ArrayList<FastSeries> grabAllSeries = new ArrayList<>();
                    for (FastParallel cmdParallel : pathsToBeCombined) {
                        for (FastCommand seriesCmd : cmdParallel.mCommands) {
                            if (seriesCmd instanceof FastSeries) {
                                grabAllSeries.add((FastSeries) seriesCmd);
                            }
                        }
                    }
            
                    ArrayList<Point> combinedListOfNewCurves = new ArrayList<>();
                    for (Curve loopingCurve : newCurves) {
                        combinedListOfNewCurves.addAll(loopingCurve.getArrayPoints());
                    }
            
                    FastParallel newPath = new FastParallel(List.of(new RunPathCommand(combinedListOfNewCurves)));
                    newPath.mCommands.addAll(grabAllSeries);
                    combinedPaths.add(newPath);
            
                    // // Check if the next path has a wait command, if not, add it to the combined path
                    // if (j < autoPaths.size() && !doesSplineContainCMDGroupsWithWaitCommands(j)) {
                    //     System.out.println("should never run");
                    //     combinedPaths.add(newPath);
                    // } else {
                    //     // If the next path has a wait command or it's the last path, add the combined path
                    //     combinedPaths.add(newPath);
                    // }
            
                    // Move i to the next path after the combined ones
                    i = j;
                }
            }

            // Replace the modified paths in autoPaths with the combined paths
            autoPaths.clear();
            autoPaths.addAll(combinedPaths);
            for(FastParallel fast : autoPaths){
                convertAutoPaths.add(fast);
                if(DebugAuto){
                    System.out.println("AutoPath " + convertAutoPaths.indexOf(fast));
                    for(FastCommand command : fast.mCommands){
                        System.out.println(command.toString());
                        if(command instanceof FastSeries){
                            FastSeries tempSeries = (FastSeries) command;
                            for(FastCommand commandsInSeries : tempSeries.mCommands){
                              System.out.println(commandsInSeries.toString());
                            }
                        }
                    }
                }
            }
            convertAutoPaths.add(new StopCommand());
            //Create the final Auto Routine
            autos.add(new FastAutoBase() {
                Point temp = curves.get(0).getArrayPoints().get(0);
                FastCommand completeAutonomousCommand = new FastSeries(convertAutoPaths);
                @Override
                public String getName() {
                    return autoFile.getName().substring(0,autoFile.getName().length()-5);
                }

                @Override
                public void routine() throws Exception {
                    runCommand(completeAutonomousCommand);
                }

                @Override
                public Pose2d getStartingPose() {
                    return new Pose2d(temp.getX(),temp.getY(),Rotation2d.fromRadians(temp.getRadians()));
                }
                
            });
        }
        autoPaths.clear();
        return autos;
    }

    public static boolean doesSplineContainCMDGroupsWithWaitCommands(int spline){
        //System.out.println("Checking path " + spline);
        if(autoPaths.size() <= spline){
            return false;
        }
        for(FastCommand command1 : autoPaths.get(spline).mCommands){
            if(command1 == null){
                return false;
            }
            if(command1 instanceof FastSeries){
                //System.out.println(command1);
                FastSeries series = (FastSeries) command1;
                for(FastCommand command2 : series.mCommands){
                    if(command2 instanceof WaitCommand
                    ){
                        //System.out.println(command2);
                        //System.out.println("Found WaitCommand in path " + spline);
                        return true;
                    }
                }
            }
        }
        return false;
    }


    public static enum CmdList {
        Wait
    }

    public static class CMD {
        public CmdList selectedCMD = CmdList.Wait;
        public double value = 0;
        public CMD(CmdList cmd, double value){
            selectedCMD = cmd;
            this.value = value;
        }
    }

    public static class CMDObject {
        public double pos = 0;
        public int spline = 0;
        public ArrayList<CMD> cmds = new ArrayList<>();
        public Point location = new Point();
        public CMDObject(ArrayList<CMD> cmds, double pos, int spline){
            this.cmds = new ArrayList<CMD>(cmds);
            this.pos = pos;
            this.spline = spline;
        }
    }

    public static class SaveData {
        private ArrayList<CurveSaveData> curves;
        private ArrayList<CMDObjectSaveData> cmdObjects;

        public SaveData(ArrayList<CurveSaveData> curves, ArrayList<CMDObjectSaveData> cmdObjects) {
            this.curves = curves;
            this.cmdObjects = cmdObjects;
        }

        public ArrayList<CurveSaveData> getCurves() {
            return curves;
        }

        public ArrayList<CMDObjectSaveData> getCmdObjects() {
            return cmdObjects;
        }
    }

    class CurveSaveData {
        private Point startPoint;
        private Point endPoint;
        private Point constraintPoint0;
        private Point constraintPoint1;

        public CurveSaveData(Point startPoint, Point endPoint, Point constraintPoint0, Point constraintPoint1) {
            this.startPoint = startPoint;
            this.endPoint = endPoint;
            this.constraintPoint0 = constraintPoint0;
            this.constraintPoint1 = constraintPoint1;
        }

        public Point getStartPoint() {
            return startPoint;
        }

        public Point getEndPoint() {
            return endPoint;
        }

        public Point getConstraintPoint0() {
            return constraintPoint0;
        }

        public Point getConstraintPoint1() {
            return constraintPoint1;
        }
    }

    class CMDObjectSaveData {
        private ArrayList<CMD> cmds;
        private int spline;
        private double pos;

        public CMDObjectSaveData(ArrayList<CMD> cmds, int spline, double pos) {
            this.cmds = cmds;
            this.spline = spline;
            this.pos = pos;
        }

        public ArrayList<CMD> getCmds() {
            return cmds;
        }

        public int getSpline() {
            return spline;
        }

        public double getPos() {
            return pos;
        }
    }
}