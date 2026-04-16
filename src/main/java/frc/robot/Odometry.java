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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import frc.robot.Constants.DriveConstants;

public class Odometry {

    private final Pigeon2 pigeon;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final AprilTagVision cam0;
    private final AprilTagVision cam1;

    private static final int pigeonID = 62;

    //wpi is weird, x is forward positive, y is left positive
    public static final Transform3d kRobotToCam0 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(2.75), Units.inchesToMeters(9.5), Units.inchesToMeters(8)),
            new Rotation3d(0, Math.toRadians(30), Math.toRadians(90))
        );
    
    public static final Transform3d kRobotToCam1 =
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-10.75), Units.inchesToMeters(-9.5), Units.inchesToMeters(7.5)),
            new Rotation3d(0, Math.toRadians(22), Math.toRadians(180))
        );

    public Odometry() {

        pigeon = new Pigeon2(pigeonID);

        //the name has to be the exact same in PhotonVision
        cam0 = new AprilTagVision("Cam0", kRobotToCam0);
        cam1 = new AprilTagVision("Cam1", kRobotToCam1);

        poseEstimator =
            new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                pigeon.getRotation2d(),
                Robot.getSwerveInstance().getModulePositions(),
                new Pose2d(),

                // wheel odometry trust std
                VecBuilder.fill(0.01, 0.01, 4.0),

                // vision trust std
                VecBuilder.fill(1.5, 1.5, 2.0)
            );
    }

    public void periodic() {

        SwerveModulePosition[] modulePositions =
            Robot.getSwerveInstance().getModulePositions();
        poseEstimator.update(
            pigeon.getRotation2d(),
            modulePositions
        ); 

        applyVision(cam0);
        applyVision(cam1);
    }

    private void applyVision(AprilTagVision cam) {

    Optional<EstimatedRobotPose> visionEstimate =
        cam.getEstimatedVisionPose();

    if (visionEstimate.isEmpty()) return;

    EstimatedRobotPose est = visionEstimate.get();
    int tagCount = est.targetsUsed.size();
    if (tagCount == 0) return;

    double avgDist = 0.0;

    for (var target : est.targetsUsed) {
        avgDist +=
            target.getBestCameraToTarget()
                  .getTranslation()
                  .getNorm();
    }

    avgDist /= tagCount;

    Matrix<N3, N1> visionStdDevs;

    if (tagCount > 1) {
            visionStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        } else {
            visionStdDevs = VecBuilder.fill(4.0, 4.0, 8.0);
        }

    // scale based on distance
    for (int i = 0; i < 3; i++) {
        visionStdDevs.set(
            i,
            0,
            visionStdDevs.get(i, 0) *
            (1 + (avgDist * avgDist / 30.0))
        );
    }
    poseEstimator.addVisionMeasurement(
        est.estimatedPose.toPose2d(),
        est.timestampSeconds,
        visionStdDevs
    );

    SmartDashboard.putString(cam.camera.getName() + " pose", est.estimatedPose.toPose2d().toString()); 
}

    public Pose2d getPose() { 
        // return poseEstimator.getEstimatedPosition();
        double pigeonDegrees = pigeon.getRotation2d().getDegrees();
        return new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(pigeonDegrees));
    } 


    public void zeroPigeon(){
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if(alliance == Alliance.Blue){
            pigeon.reset();
            poseEstimator.resetPosition(Rotation2d.fromDegrees(0), 
            Robot.getSwerveInstance().getModulePositions(),
            new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(0)));
        } else {
            pigeon.setYaw(180);
            poseEstimator.resetPosition(Rotation2d.fromDegrees(180), 
            Robot.getSwerveInstance().getModulePositions(),
            new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(180)));
        }
    }

    public Rotation2d getHeading() {
        return pigeon.getRotation2d();
    }

    public ChassisSpeeds getFieldRelativeSpeeds(){
        return ChassisSpeeds.fromRobotRelativeSpeeds(Robot.getSwerveInstance().getRobotRelativeSpeeds(), getHeading());
    }

    public void resetPose(Pose2d pose) {
        pigeon.setYaw(pose.getRotation().getDegrees());
        poseEstimator.resetPosition(pose.getRotation(),
        Robot.getSwerveInstance().getModulePositions(),
        pose);
    }

    public void setPigeonAngle(double rot){
        pigeon.setYaw(rot);
    }

    public boolean isCam0Connected() {
        return cam0.isConnected();
    }

    public boolean isCam1Connected() {
        return cam1.isConnected();
    }

    private static class AprilTagVision {

        private final PhotonCamera camera;
        private final PhotonPoseEstimator estimator;

        private final AprilTagFieldLayout tagLayout =
            AprilTagFieldLayout.loadField(
                AprilTagFields.k2026RebuiltWelded
            );

        public AprilTagVision(
            String name,
            Transform3d robotToCam
        ) {

            camera = new PhotonCamera(name);

            estimator =
                new PhotonPoseEstimator(
                    tagLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCam
                );

            estimator.setMultiTagFallbackStrategy(
                PoseStrategy.LOWEST_AMBIGUITY
            );

            estimator.setTagModel(TargetModel.kAprilTag36h11);
        }

        public boolean isConnected() {
            return camera.isConnected();
        }

        public Optional<EstimatedRobotPose> getEstimatedVisionPose() {

            if (!camera.isConnected())
                return Optional.empty();

            var result = camera.getLatestResult();
            SmartDashboard.putBoolean(camera.getName() + " has targets?", result.hasTargets());

            

            if (!result.hasTargets())
                return Optional.empty();
            var estimate = estimator.update(result);

            SmartDashboard.putBoolean(camera.getName() + " pose present?", estimate.isPresent());

            return estimate;
        }
    }
    
}
