package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.DriveConstants;

public class Odometry {

    private final Pigeon2 pigeon;
    private final SwerveDriveOdometry odometry;

    public Odometry() {
        // Initialize gyro
        pigeon = new Pigeon2(OdometryConstants.pigeonID);
        pigeon.reset();

        // Initialize odometry with current heading and module positions
        SwerveModulePosition[] modulePositions = Robot.getSwerveInstance().getModulePositions();

        odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            getHeading(),
            modulePositions,
            new Pose2d(0, 0, new Rotation2d()) // Start at origin
        );
    }

    public void periodic() {
        SwerveModulePosition[] modulePositions = Robot.getSwerveInstance().getModulePositions();
        odometry.update(getHeading(), modulePositions);
    }

    public Pose2d getPose() {
        return new Pose2d(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), getHeading());
    }

    public Rotation2d getHeading() {
        return pigeon.getRotation2d();
    }

    public void resetPose(Pose2d pose) {
        SwerveModulePosition[] modulePositions = Robot.getSwerveInstance().getModulePositions();
        odometry.resetPosition(getHeading(), modulePositions, pose);
    }

    private static final class OdometryConstants{
        private static final int pigeonID = 62;
    }

}
