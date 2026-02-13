package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.Constants.DriveConstants;

public class Odometry {

    private final Pigeon2 pigeon;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final AprilTagVision cam0;
    private final AprilTagVision cam1;

    // Camera-to-robot transforms
    public static final Transform3d kRobotToCam0 =
            new Transform3d(
                    new Translation3d(-0.23, 0.18, 0.34),
                    new Rotation3d(0, Math.toRadians(7.5), Math.toRadians(180))
            );

    public static final Transform3d kRobotToCam1 =
            new Transform3d(
                    new Translation3d(-0.20, -0.18, 0.34),
                    new Rotation3d(0, Math.toRadians(7.5), Math.toRadians(180))
            );

    private static final int pigeonID = 62;

    public Odometry() {

        pigeon = new Pigeon2(pigeonID);
        pigeon.reset();

        cam0 = new AprilTagVision("Camera0", kRobotToCam0);
        cam1 = new AprilTagVision("Camera1", kRobotToCam1);

        // Initialize pose estimator with old code std devs
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        DriveConstants.kDriveKinematics,
                        pigeon.getRotation2d(),
                        Robot.getSwerveInstance().getModulePositions(),
                        new Pose2d(),
                        VecBuilder.fill(0.05, 0.05, 1.0), // odometry std devs
                        VecBuilder.fill(9.0, 9.0, 9.0)     // vision std devs default
                );
    }

    public void periodic() {
        // Update odometry from module positions + gyro
        SwerveModulePosition[] modulePositions = Robot.getSwerveInstance().getModulePositions();
        poseEstimator.update(pigeon.getRotation2d(), modulePositions);

        // Update vision measurements
        cam0.updateCamera();
        cam1.updateCamera();

        applyVision(cam0);
        applyVision(cam1);
    }

    private void applyVision(AprilTagVision cam) {
        Optional<EstimatedRobotPose> visionEstimate = cam.getEstimatedVisionPose();
        if (visionEstimate.isEmpty()) return;

        EstimatedRobotPose est = visionEstimate.get();
        int tagCount = est.targetsUsed.size();

        if (tagCount == 0) return;

        // Compute average distance to tags
        double avgDist = 0.0;
        for (var target : est.targetsUsed) {
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDist /= tagCount;

        // Determine vision std devs (3x1 matrix)
        Matrix<N3, N1> visionStdDevs;
        if (tagCount > 1) {
            visionStdDevs = VecBuilder.fill(1.2, 1.2, 1.2);
        } else {
            visionStdDevs = VecBuilder.fill(9.0, 9.0, 9.0);
        }

        // Distance scaling (like old heuristic)
        for (int i = 0; i < 3; i++) {
            visionStdDevs.set(i, 0, visionStdDevs.get(i, 0) * (1 + (avgDist * avgDist / 30.0)));
        }

        // Add measurement to estimator
        poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, visionStdDevs);
    }

    // ===============================
    // ===== PUBLIC ACCESSORS ========
    // ===============================
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return pigeon.getRotation2d();
    }

    public void resetPose(Pose2d pose) {
        SwerveModulePosition[] modulePositions = Robot.getSwerveInstance().getModulePositions();
        poseEstimator.resetPosition(pigeon.getRotation2d(), modulePositions, pose);
    }

    public boolean areCamerasConnected() {
        return cam0.isConnected() && cam1.isConnected();
    }

    // ===============================
    // ===== AprilTag Vision Class ===
    // ===============================
    private static class AprilTagVision {

        private final PhotonCamera camera;
        private final PhotonPoseEstimator estimator;

        private final AprilTagFieldLayout tagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public AprilTagVision(String name, Transform3d robotToCam) {
            camera = new PhotonCamera(name);
            estimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
            estimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
            estimator.setTagModel(TargetModel.kAprilTag36h11);
        }

        public void updateCamera() {
            // No buffer needed here; just latest result
        }

        public boolean isConnected() {
            return camera.isConnected();
        }

        public Optional<EstimatedRobotPose> getEstimatedVisionPose() {
            if (!camera.isConnected()) return Optional.empty();
            var result = camera.getLatestResult();
            if (!result.hasTargets()) return Optional.empty();
            return estimator.update(result);
        }
    }
}
