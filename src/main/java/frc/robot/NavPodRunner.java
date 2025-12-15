package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class NavPodRunner {
    NavPod _navpod;
    double kx, ky, kgx, kgy, kgz, gyroRotation;

    public NavPodRunner(){
        if(!Robot.isReal()){
            return;
        }
        _navpod = new NavPod();
        if (_navpod.isValid()) {
            NavPodConfig config = new NavPodConfig();
            config.cableMountAngle = 90;
            config.fieldOrientedEnabled = true;
            config.initialHeadingAngle = 0;
            config.mountOffsetX = 0;
            config.mountOffsetY = -1.5;
            config.rotationScaleFactorX = -0.007; // 0.0675
            config.rotationScaleFactorY = 0.03; // 0.02
            config.translationScaleFactor = 0.00804137; // 0.008567
            _navpod.setConfig(config);
            
            // Report values to the console
            config = _navpod.getConfig();
            System.err.printf("config.cableMountAngle: %f\n", config.cableMountAngle);
            System.err.printf("config.fieldOrientedEnabled: %b\n", config.fieldOrientedEnabled);
            System.err.printf("config.initialHeadingAngle: %f\n", config.initialHeadingAngle);
            System.err.printf("config.mountOffsetX: %f in\n", config.mountOffsetX);
            System.err.printf("config.mountOffsetY: %f in\n", config.mountOffsetY);
            System.err.printf("config.rotationScaleFactorX: %f\n", config.rotationScaleFactorX);
            System.err.printf("config.rotationScaleFactorY: %f\n", config.rotationScaleFactorY);
            System.err.printf("config.translationScaleFactor: %f\n", config.translationScaleFactor);
            
            _navpod.resetH(0);
            _navpod.resetXY(0, 0);
            
            _navpod.setAutoUpdate(0.02, update -> {gyroRotation = update.h; kx = update.x; ky = update.y; kgx = update.gx; kgy = update.gy; kgz = update.gz;});
        }
        else{
            System.err.println("Error Navpod Could not be found! Check Wiring and reboot!");
        }
    }
    public void setH(double h){
        if(_navpod != null){
            _navpod.resetH(h);
        }
        if(Robot.isSimulation()){
            gyroRotation = h;
        }
    }
    public void setXY(double x, double y){
        if(_navpod != null){
            _navpod.resetXY(-x,-y);
        } 
        if(Robot.isSimulation()){
            kx = -x;
            ky = -y;
        }  
    }
    public Pose2d getPose(){
        return new Pose2d(-kx,-ky, Rotation2d.fromDegrees(gyroRotation));
    }
}
