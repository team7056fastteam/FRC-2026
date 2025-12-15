package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class FastPoseEstimator {
    AprilTagVision kurtVision0;
    AprilTagVision kurtVision1;
    private Pose2d estimatedPose = new Pose2d();
    private Pose2d lastNavPodPose = new Pose2d();

    private static final int BufferLength = 75;
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, 1.0);
    ArrayList<Pose2dTimeStamp> poseBuffer = new ArrayList<>();

    public static final Transform3d kRobotToCam0 =
            new Transform3d(new Translation3d(Units.inchesToMeters(-9), Units.inchesToMeters(7), Units.inchesToMeters(13.5)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(7.5), Math.toRadians(181)));

    public static final Transform3d kRobotToCam1 =
            new Transform3d(new Translation3d(Units.inchesToMeters(-8), Units.inchesToMeters(-7), Units.inchesToMeters(13.5)),
            new Rotation3d(Math.toRadians(3), Math.toRadians(7.5), Math.toRadians(180)));

    public FastPoseEstimator(Pose2d initalPose){
        kurtVision0 = new AprilTagVision("KurtCamera", stateStdDevs, kRobotToCam0);
        kurtVision1 = new AprilTagVision("KurtCamera1", stateStdDevs, kRobotToCam1);
        estimatedPose = new Pose2d(initalPose.getTranslation(),initalPose.getRotation());
        lastNavPodPose = new Pose2d(initalPose.getTranslation(),initalPose.getRotation());
    }

    public void setInitalPose(Pose2d initalPose){
        estimatedPose = new Pose2d(initalPose.getTranslation(),initalPose.getRotation());
        lastNavPodPose = new Pose2d(initalPose.getTranslation(),initalPose.getRotation());
        
        updatePoseNavPod(initalPose);
        Robot.ResetNavPod(initalPose);
    }
    Pose2d KurtCameraRobotPose = new Pose2d();
    public Pose2d getKurtCameraRobotPose(){
        Optional<EstimatedRobotPose> estimatedRobotPose = kurtVision0.getEstimatedVisionPose();
        if(estimatedRobotPose.isPresent()){
            KurtCameraRobotPose = convertWPIPoseToKurtPose(estimatedRobotPose.get().estimatedPose.toPose2d());
        }

        return KurtCameraRobotPose;
    }
    Pose2d KurtCamera1RobotPose = new Pose2d();
    public Pose2d getKurtCamera1RobotPose(){
        Optional<EstimatedRobotPose> estimatedRobotPose = kurtVision1.getEstimatedVisionPose();
        if(estimatedRobotPose.isPresent()){  
            KurtCamera1RobotPose = convertWPIPoseToKurtPose(estimatedRobotPose.get().estimatedPose.toPose2d());
        }
        return KurtCamera1RobotPose;
    }
    public void updateCameras(){
        kurtVision0.updateCamera();
        kurtVision1.updateCamera();
    }

    public void updatePoseNavPod(Pose2d navPod){
        Pose2d poseChange = new Pose2d(navPod.getX() - lastNavPodPose.getX(),navPod.getY() - lastNavPodPose.getY(),navPod.getRotation());
        estimatedPose = new Pose2d(estimatedPose.getX() + poseChange.getX(),estimatedPose.getY() + poseChange.getY(),navPod.getRotation());
        bufferPose(new Pose2dTimeStamp(estimatedPose, Timer.getFPGATimestamp()));
        lastNavPodPose = navPod;
    }
    
    public void CheckVisionMeasurementAndSendUpdate(){
        Optional<EstimatedRobotPose> visionPose0 = kurtVision0.getEstimatedVisionPose();
        Optional<EstimatedRobotPose> visionPose1 = kurtVision1.getEstimatedVisionPose();

        if(visionPose0.isPresent()){
            kurtVision0.calculateVisionMeasurementStdDevs();
            UpdatePoseWithVision(convertWPIPoseToKurtPose(visionPose0.get().estimatedPose.toPose2d()),kurtVision0.getVisionMatrix(),visionPose0.get().timestampSeconds);
        }
        
        if(visionPose1.isPresent()){
            kurtVision1.calculateVisionMeasurementStdDevs();
            UpdatePoseWithVision(convertWPIPoseToKurtPose(visionPose1.get().estimatedPose.toPose2d()),kurtVision1.getVisionMatrix(),visionPose1.get().timestampSeconds);
        }
    }

    public void UpdatePoseWithVision(Pose2d visionRobotPoseMeters, Matrix<N3, N3> m_visionK, double timestampSeconds){
        if(timestampSeconds < (Timer.getFPGATimestamp()-((BufferLength*20)/1000))){
            System.out.println("tag data is old");
            return; //exit if vision is old
        }
        //get nearest timestamp to vision event
        double closestTimeStamp = Double.POSITIVE_INFINITY;
        Pose2dTimeStamp closestPose2dTimeStamp = new Pose2dTimeStamp(new Pose2d(), -1);

        for(Pose2dTimeStamp pose : poseBuffer){
            if(closestTimeStamp >= Math.abs(pose.timeStamp - timestampSeconds)){
                closestPose2dTimeStamp = pose;
            }
        }

        if(closestPose2dTimeStamp.timeStamp < 0){
            System.out.println("closest pose is impossibly new");
            return;
        }

        double visionPoseRotations = visionRobotPoseMeters.getRotation().getRotations();
        visionPoseRotations = visionPoseRotations > 0 ? visionPoseRotations : visionPoseRotations + 1;
        Rotation2d rotationChange = Rotation2d.fromRotations(visionPoseRotations - closestPose2dTimeStamp.pose.getRotation().getRotations());
        // System.out.println("Wanted Rotation: " + visionPoseRotations*360 + " Error: " + rotationChange.getDegrees());
        // rotationChange = rotationChange.getRotations() > 0.5 ? Rotation2d.fromRotations(rotationChange.getRotations() - 1) : rotationChange; //might need this
        // System.out.println("New Error: " + rotationChange.getDegrees());
        Pose2d poseChange = new Pose2d(visionRobotPoseMeters.getX() - closestPose2dTimeStamp.pose.getX(), visionRobotPoseMeters.getY() - closestPose2dTimeStamp.pose.getY(), rotationChange);

        var k_times_pose = m_visionK.times(VecBuilder.fill(poseChange.getX(), poseChange.getY(), rotationChange.getRadians()));
        var scaledPose =
            new Pose2d(k_times_pose.get(0, 0), k_times_pose.get(1, 0), Rotation2d.fromRadians(k_times_pose.get(2, 0)));
        estimatedPose = new Pose2d(estimatedPose.getX() + scaledPose.getX(),estimatedPose.getY() + scaledPose.getY(), estimatedPose.getRotation());
        // System.out.println("Pose Updated (Old Pose: " + closestPose2dTimeStamp.pose.toString() + " New Pose: " + visionRobotPoseMeters.toString() + ")");

        bufferPose(new Pose2dTimeStamp(estimatedPose, Timer.getFPGATimestamp()));
        
    }

    Pose2d convertWPIPoseToKurtPose(Pose2d initalPose){
        return new Pose2d(Units.metersToInches(8.05 - initalPose.getY()), Units.metersToInches(initalPose.getX()), initalPose.getRotation());
    }

    public Pose2d getEstimatedPose(){
        return estimatedPose;
    }

    public void bufferPose(Pose2dTimeStamp poseToBeBuffered){
        poseBuffer.add(poseToBeBuffered);
        if(poseBuffer.size() > BufferLength){
            for(int i = 0; i < (poseBuffer.size() - BufferLength); i++){
                poseBuffer.remove(0);
            }
        }
    }

    public boolean isCamerasConnected(){
        return kurtVision0.isConnected() && kurtVision1.isConnected();
    }
}

class AprilTagVision {
    private Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
    private Matrix<N3, N3> m_visionK = new Matrix<>(Nat.N3(), Nat.N3());

    private PhotonCamera kurtCamera;
    private PhotonPoseEstimator photonEstimator;
    private String name;

    List<PhotonPipelineResult> lastestResults;
    
    public static Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(9, 9, 9);
    public static Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(1.2, 1.2, 1.2);

    private Matrix<N3, N1> curStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
    AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    static final boolean printIfCamerasAreConnected = false;

    public AprilTagVision(String name, Matrix<N3, N1> stateStdDevs, Transform3d kRobotToCam) {
        kurtCamera = new PhotonCamera(name);
        this.name = name;

        photonEstimator =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        photonEstimator.setTagModel(TargetModel.kAprilTag36h11);

        for (int i = 0; i < 3; ++i) {
            m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
    }

    public boolean isConnected(){
        return kurtCamera.isConnected();
    }

    public void updateCamera(){
        if(kurtCamera.isConnected()){
            lastestResults = kurtCamera.getAllUnreadResults();
        }
        else{
            if(printIfCamerasAreConnected){
                System.err.println("Camera (" + name + ") is not Connected!");
            }
        }
    }

    public List<PhotonPipelineResult> getLastestResult(){
        if(kurtCamera.isConnected()){
            return lastestResults;
        }

        return null;
    }

    public Optional<EstimatedRobotPose> getEstimatedVisionPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        if(getLastestResult() != null){
            for (var change : getLastestResult()) {
                if(!change.hasTargets() || change == null){
                    continue;
                }
                visionEst = photonEstimator.update(change);
                
                updateEstimationStdDevs(visionEst, change.getTargets());
            }
        }
        return visionEst;
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N3> getVisionMatrix() {
        return m_visionK;
    }

    public final void calculateVisionMeasurementStdDevs() {
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
          r[i] = curStdDevs.get(i, 0) * curStdDevs.get(i, 0);
        }
    
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        for (int row = 0; row < 3; ++row) {
          if (m_q.get(row, 0) == 0.0) {
            m_visionK.set(row, row, 0.0);
          } else {
            m_visionK.set(
                row, row, m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row])));
          }
        }
    }
}

class Pose2dTimeStamp {
    Pose2d pose;
    double timeStamp;

    Pose2dTimeStamp(Pose2d pose, double timeStamp){
        this.pose = pose;
        this.timeStamp = timeStamp;
    }

    public String toString(){
        return "Pose2dTimeStamp(pose: " + pose.toString() + " TimeStamp: " + timeStamp;
    }
}