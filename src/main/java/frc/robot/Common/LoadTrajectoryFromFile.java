package frc.robot.Common;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.ArrayList;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class LoadTrajectoryFromFile {
    static File folder = new File(Filesystem.getDeployDirectory().getPath()+"/Paths/");

    public static FastTrajectory load(String Path) {
        try{
            ArrayList<CurveSaveData> loadedCurves = new ArrayList<>();
            ArrayList<Curve> curves = new ArrayList<>();
            ArrayList<Point> points = new ArrayList<>();
            SaveData saveData;
            Gson gson = new Gson();
            Type saveDataType = new TypeToken<SaveData>() {}.getType();
            try (FileReader reader = new FileReader(folder.getAbsolutePath() + "/" + Path)) {
                saveData = gson.fromJson(reader, saveDataType);
            }

            loadedCurves = saveData.getCurves();

            for (CurveSaveData curveData : loadedCurves) {
                Curve curve = new Curve(
                        curveData.getStartPoint(),
                        curveData.getEndPoint(),
                        curveData.getConstraintPoint0(),
                        curveData.getConstraintPoint1()
                );
                curves.add(curve);
            }

            for(Curve curve : curves){
                points.addAll(curve.getArrayPoints());
            }

            return new FastTrajectory(KurtMath.convertPointListToPose2d(points), Constants.AutoConstants.kMaxSpeedInchesPerSecond, Constants.AutoConstants.kMaxAccelerationInchesPerSecondSquared);
        }
        catch(IOException e){
            e.printStackTrace();
        }
        System.out.println("stunned");
        return null;
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

    public static class SaveData {
        private ArrayList<CurveSaveData> curves;

        public SaveData(ArrayList<CurveSaveData> curves) {
            this.curves = curves;
        }

        public ArrayList<CurveSaveData> getCurves() {
            return curves;
        }
    }
}
