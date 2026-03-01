// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Auto.Ingest;
import frc.robot.Auto.PathLoader;
import frc.robot.Auto.ShootForTime;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Kicker;
// import frc.robot.Subsystems.LedSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Spindexer;
import frc.robot.Subsystems.SwerveSubsystem;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static Teleop _teleop;
  private static Odometry _odometry;
  private static SwerveSubsystem _swerve;
  private static Intake _intake;
  private static Spindexer _spindexer;
  private static Shooter _shooter;
  private static Kicker _kicker;
  // private static LedSubsystem _led;
  private PathLoader pathLoader;
  private Command autonomousCommand;
  private Field2d field;



  @Override
  public void robotInit() {
    _teleop = new Teleop();
    _swerve = getSwerveInstance();
    _odometry = getOdometryInstance();
    _intake = getIntakeInstance();
    _kicker = getKickerInstance();
    _spindexer = getSpindexerInstance();
    _shooter = getShooterInstance();
    // _led = getLedInstance();
    pathLoader = new PathLoader();
    field = new Field2d();
    registerNamedCommands();
  }

  @Override
  public void robotPeriodic() {
    _odometry.periodic();
    CommandScheduler.getInstance().run();
    RobotDashboard();
    field.setRobotPose(_odometry.getPose());
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = pathLoader.getSelectedAuto();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    _teleop.TeleopInit();
  }

  @Override
  public void teleopPeriodic() {
    _teleop.Driver();
    _teleop.Operator();
    _teleop.Dashboard();
  }

  @Override
  public void disabledInit() {
    _intake.stop();
    _kicker.stop();
    _shooter.stop();
    _spindexer.stop();
    _swerve.stop();
  }

  @Override
  public void disabledPeriodic() {
    if(pathLoader.getSelectedAuto() instanceof PathPlannerAuto auto){

      Pose2d startPose = auto.getStartingPose();

      if(AutoBuilder.shouldFlip()){
        startPose = FlippingUtil.flipFieldPose(startPose);
      }

      SmartDashboard.putString("Robot Starting Pose Inches", new Pose2d(
        new Translation2d(Units.metersToInches(startPose.getX()), Units.metersToInches(startPose.getY())),
        new Rotation2d(startPose.getRotation().getRadians()))
            .toString()
        );
      SmartDashboard.putString("Robot Starting Pose Meters", startPose.toString());
    }
  }

  public void RobotDashboard(){
    dashboard();
    SmartDashboard.putNumber("Heading",_odometry.getHeading().getDegrees() % 360);
    SmartDashboard.putString("Robot Pose", _odometry.getPose().toString());
    SmartDashboard.putData("Field", field);
  }


  public static void ResetOdometry(Pose2d pose){
    _odometry.resetPose(pose);
  }

  public static Rotation2d getGyroscopeRotation2d() {
    if(_odometry == null){
      return Rotation2d.fromDegrees(0);
    }
    return _odometry.getHeading();
  }


  public void dashboard(){
    _intake.dashboard();
    _spindexer.dashboard();
    _kicker.dashboard();
    _shooter.dashboard();
    // _led.dashboard();
    _swerve.dashboard();
  }

  public void registerNamedCommands(){
    NamedCommands.registerCommand("ingest", new Ingest(Intake.IntakeState.Forward));
    NamedCommands.registerCommand("unIngest", new Ingest(Intake.IntakeState.Idle));
    NamedCommands.registerCommand("shootClose", new ShootForTime(Shooter.ShooterState.Close, 5.0));
    NamedCommands.registerCommand("shootMid", new ShootForTime(Shooter.ShooterState.Mid, 5.0));
    NamedCommands.registerCommand("shootFar", new ShootForTime(Shooter.ShooterState.Far, 5.0));
  }

  public static SwerveSubsystem getSwerveInstance(){
    if(_swerve == null){
      _swerve = new SwerveSubsystem();
    }

    return _swerve;
  }

  public static Intake getIntakeInstance(){
    if(_intake == null){
      _intake = new Intake();
    }
    return _intake;
  }

  public static Spindexer getSpindexerInstance(){
    if(_spindexer == null){
      _spindexer = new Spindexer();
    }
    return _spindexer;
  }

  public static Kicker getKickerInstance(){
    if(_kicker == null){
      _kicker = new Kicker();
    }
    return _kicker;
  }

  public static Shooter getShooterInstance(){
    if(_shooter == null){
      _shooter = new Shooter();
    }
    return _shooter;
  }

  // public static LedSubsystem getLedInstance(){
  //   if(_led == null){
  //     _led = new LedSubsystem();
  //   }
  //   return _led;
  // }

  public static Odometry getOdometryInstance(){
    if(_odometry == null){
      _odometry = new Odometry();
    }
    return _odometry;
  }

  public static void setPose(Pose2d pose2d) {
    _odometry.resetPose(pose2d);
  }

}