// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.KurtLogger.logType;
import frc.robot.Autos.AutoModeSelector;
import frc.robot.Common.AutoModeRunner;
import frc.robot.Common.FastAutoBase;
import frc.robot.Subsystems.SubsystemManager;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static AutoModeRunner _autoModeRunner;
  private static AutoModeSelector _modeSelector;
  private static Teleop _teleop;
  private static NavPodRunner _navPod;
  private static SubsystemManager _manager;
  public static FastPoseEstimator _poseEstimator;
  
  static Pose2d currentPose = new Pose2d();

  private static KurtLogger autoLogger;
  private static KurtLogger teleopLogger;

  @Override
  public void robotInit() {
    _teleop = new Teleop();
    _autoModeRunner = new AutoModeRunner();
    _modeSelector = new AutoModeSelector();
    _poseEstimator = new FastPoseEstimator(currentPose);
    _navPod = new NavPodRunner();
    _manager = SubsystemManager.getInstance();
    _manager.registerSubsystems();
    _manager.init();
  }

  @Override
  public void robotPeriodic() {
    _manager.dashboard();
    _poseEstimator.updatePoseNavPod(_navPod.getPose());
    currentPose = _poseEstimator.getEstimatedPose();
    currentPose = new Pose2d(currentPose.getTranslation(),getGyroscopeRotation2d());
    _poseEstimator.updateCameras();

    RobotDashboard();
  }

  @Override
  public void autonomousInit() {
    autoLogger = new KurtLogger("Auto");
    autoLogger.initLog();
    _manager.setLogger(autoLogger);
    _manager.init();

    if(_modeSelector.getAutoMode() != null){
      _poseEstimator.setInitalPose(_modeSelector.getAutoMode().getStartingPose());
      _autoModeRunner.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    _manager.run();
    _poseEstimator.CheckVisionMeasurementAndSendUpdate();
    // autoLogger.logData(logType.robotPose, currentPose.toString(), "Robot");
    // autoLogger.logData(logType.robotPose, _navPod.getPose().toString(), "NavPod");
  }

  @Override
  public void teleopInit() {
    teleopLogger = new KurtLogger("TeleOp");
    teleopLogger.initLog();
    _manager.setLogger(teleopLogger);
    _manager.init();
    _autoModeRunner.stop();
    _teleop.TeleopInit();
  }

  @Override
  public void teleopPeriodic() {
    _poseEstimator.CheckVisionMeasurementAndSendUpdate();
    _manager.run();
    _teleop.Driver();
    // _teleop.Operator();
    _teleop.Dashboard();
    // teleopLogger.logData(logType.robotPose, currentPose.toString(), "Robot");
  }

  @Override
  public void disabledInit() {
    _modeSelector.resetAutos();
    _manager.stop();
  }

  @Override
  public void disabledPeriodic() {
    if(_modeSelector.getAutoMode() != null && _autoModeRunner != null){
      _autoModeRunner.stop();
      FastAutoBase auto = _modeSelector.getAutoMode();
      _autoModeRunner.setAuto(auto);
    }

    try {
      teleopLogger.close();
      autoLogger.close();
    } catch (Exception e) {}
  }

  public void RobotDashboard(){
    SmartDashboard.putString("Robot Location", currentPose.toString());
    SmartDashboard.putString("NavPod Location", _navPod.getPose().toString());
    SmartDashboard.putNumber("RobotX", currentPose.getX());
    SmartDashboard.putNumber("RobotY", currentPose.getY());
    SmartDashboard.putNumber("NavPodX", _navPod.getPose().getX());
    SmartDashboard.putNumber("NavPodY", _navPod.getPose().getY());
    SmartDashboard.putNumber("RobotH", currentPose.getRotation().getRadians());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("IsEnabled", DriverStation.isEnabled());

    if(DriverStation.isDisabled()){
      SmartDashboard.putString("Robot Location According to Kurt Cam0", _poseEstimator.getKurtCameraRobotPose().toString());
      SmartDashboard.putString("Robot Location According to Kurt Cam1", _poseEstimator.getKurtCamera1RobotPose().toString());
    }
  }

  public static Pose2d getPose(){
    return currentPose;
  }
  public static void ResetNavPod(Pose2d pose){
    _navPod.setXY(pose.getX(), pose.getY());
    _navPod.setH(pose.getRotation().getDegrees());
  }

  public static Rotation2d getGyroscopeRotation2d() {
    if(_navPod == null){
      return Rotation2d.fromDegrees(0);
    }
    return Rotation2d.fromDegrees(_navPod.gyroRotation);
  }

  public static void setPose(Pose2d pose){
    _poseEstimator.setInitalPose(pose);
  }

  public static KurtLogger getAutoLogger(){
    return autoLogger;
  }
  public static KurtLogger getTeleopLogger(){
    return teleopLogger;
  }

  public static boolean doesRobotHaveCameras(){
    return _poseEstimator.isCamerasConnected();
  }

  public static void ModifyPoseFromSpeed(ChassisSpeeds speed){
    double vX = -speed.vyMetersPerSecond;
    double vY = speed.vxMetersPerSecond;
    double vH = speed.omegaRadiansPerSecond;
    var twist = new Twist2d(vX,vY,vH/32);
    Pose2d newPose = _navPod.getPose().exp(twist);
    double angleChange = newPose.getRotation().getDegrees();
    if(angleChange < 0){
      angleChange = angleChange + 360;
    }
    _navPod.setXY(newPose.getX(), newPose.getY());
    _navPod.setH(angleChange);
    _poseEstimator.updatePoseNavPod(newPose);
  }
}