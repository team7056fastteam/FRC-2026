package frc.robot.Auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Robot;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Odometry;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

public class PathLoader {

    private final SwerveSubsystem swerve = Robot.getSwerveInstance();
    private final Odometry odometry = Robot.getOdometryInstance();
    private SendableChooser<Command> autoChooser;

    public PathLoader() {
        RobotConfig robotConfig;

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load RobotConfig", e);
        }

        // Controller REQUIRED for 2026
        PPHolonomicDriveController controller =
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)
                );

        BooleanSupplier mirrorPath =
                () -> DriverStation.getAlliance()
                        .orElse(DriverStation.Alliance.Blue)
                        == DriverStation.Alliance.Red;

        // Configure AutoBuilder
        AutoBuilder.configure(
                odometry::getPose,                  // Supplier<Pose2d>
                odometry::resetPose,                // Consumer<Pose2d>
                swerve::getRobotRelativeSpeeds,     // Supplier<ChassisSpeeds>
                swerve::drive,                      // BiConsumer<ChassisSpeeds, DriveFeedforwards>
                controller,                         // PathFollowingController
                robotConfig,
                mirrorPath,
                swerve
        );

        // Automatically setup autos from deployed PathPlanner autos
        setupAutos();
    }

    private void setupAutos() {
        // Build a SendableChooser with all deployed autos
        autoChooser = AutoBuilder.buildAutoChooser();

        // Put chooser on SmartDashboard
        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}
