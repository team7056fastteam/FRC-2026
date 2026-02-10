package frc.robot.Autos;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Robot;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.IntakePivot;
import frc.robot.AutoCommands.ShootForTime;
import frc.robot.AutoCommands.Ingest;
import frc.robot.Odometry;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

public class PathLoader {

    private final SwerveSubsystem _swerve = Robot.getSwerveInstance();
    private final Odometry _odometry = Robot.getOdometryInstance();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final RobotConfig robotConfig;

    public PathLoader() {
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load RobotConfig from GUI");
        }

        setupAutos();
    }

    private void setupAutos() {
        autoChooser.setDefaultOption("Example Auto", buildAutoCommand("ExamplePath"));
        autoChooser.addOption("Five Ball Auto", buildAutoCommand("FiveBallPath"));
        autoChooser.addOption("Two Ball Auto", buildAutoCommand("TwoBallPath"));

        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    private Command buildAutoCommand(String pathName) {
        // Event map: markers in path JSON -> commands
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("shootShort", new ShootForTime(Shooter.ShooterState.Close, 2.0));
        eventMap.put("shootMid", new ShootForTime(Shooter.ShooterState.Mid, 3.0));
        eventMap.put("shootLong", new ShootForTime(Shooter.ShooterState.Far, 4.0));
        eventMap.put("ingest", new Ingest(Intake.IntakeState.Forward, IntakePivot.IntakePivotState.Down));

        // Determine if path should be mirrored for Red alliance
        boolean isRed = DriverStation.getAlliance() == DriverStation.Alliance.Red;

        // AutoBuilder constructor from PathPlanner 2024-2025
        AutoBuilder autoBuilder = new AutoBuilder(
                _odometry.getPose,   // pose supplier
                _swerve::getRobotRelativeSpeeds,  // chassis speeds supplier
                _swerve::drive,                    // drive method
                robotConfig,                       // robot config
                eventMap,                          // event map
                isRed,                             // mirror path for red alliance
                _swerve                             // required subsystem
        );

        return autoBuilder.buildAuto(pathName); // instance method
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}
