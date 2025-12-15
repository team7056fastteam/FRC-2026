package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Common.ControllerFunction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.SwerveSubsystem.SwerveState;

public class Teleop {
    SubsystemManager _manager = SubsystemManager.getInstance();
    SwerveSubsystem _drive;

    XboxController driver = new XboxController(0);
    XboxController operator = new XboxController(1);

    ControllerFunction get;
    double xT = 1, rT = 1, driveX, driveY, driveZ, z;

    AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    double xPowerOffset;
    double yPowerOffset;
    
    double kTagP = 0.1;
    double poleOffset = 7;
    double robotOffset = 17.5;
    double maxTagPower = 2;
    double closeMaxTagPower = 0.85;
    double closeTagDist = 45;
    Translation2d adjustment = new Translation2d();

    int closestReefTag = -1;

    Map<Integer, Double> BackHeadingTag = new TreeMap<>();
    Map<Integer, Double> FrontHeadingTag = new TreeMap<>();

    Boolean up, down, right, left;

    public void TeleopInit(){

        get = new ControllerFunction(driver, operator);

        _drive.setState(SwerveState.TeleOp);
        BackHeadingTag.put(6, Math.toRadians(300));
        BackHeadingTag.put(7, Math.toRadians(0));
        BackHeadingTag.put(8, Math.toRadians(60));
        BackHeadingTag.put(9, Math.toRadians(120));
        BackHeadingTag.put(10, Math.toRadians(180));
        BackHeadingTag.put(11, Math.toRadians(240));
        BackHeadingTag.put(17, Math.toRadians(240));
        BackHeadingTag.put(18, Math.toRadians(180));
        BackHeadingTag.put(19, Math.toRadians(120));
        BackHeadingTag.put(20, Math.toRadians(60));
        BackHeadingTag.put(21, Math.toRadians(0.0));
        BackHeadingTag.put(22, Math.toRadians(300));

        FrontHeadingTag.put(6, Math.toRadians(120));
        FrontHeadingTag.put(7, Math.toRadians(180));
        FrontHeadingTag.put(8, Math.toRadians(240));
        FrontHeadingTag.put(9, Math.toRadians(300));
        FrontHeadingTag.put(10, Math.toRadians(0));
        FrontHeadingTag.put(11, Math.toRadians(60));
        FrontHeadingTag.put(17, Math.toRadians(60));
        FrontHeadingTag.put(18, Math.toRadians(0));
        FrontHeadingTag.put(19, Math.toRadians(300));
        FrontHeadingTag.put(20, Math.toRadians(240));
        FrontHeadingTag.put(21, Math.toRadians(180));
        FrontHeadingTag.put(22, Math.toRadians(120));
    }

    public void Driver(){
        if(!driver.isConnected()){ return; }

        get.isPressed(get.speedAdjustment(),()-> {xT = 1.4;});
        get.isNotPressed(get.speedAdjustment(),()-> {xT = 0.675;});

        get.isPressed((get.driverLeftTrigger() || get.driverRightTrigger()) && getClosestReefTag() != -1 && scoreFront(), ()-> _drive.snapHeading(FrontHeadingTag.get(getClosestReefTag())));
        get.isPressed((get.driverLeftTrigger() || get.driverRightTrigger()) && getClosestReefTag() != -1 && !scoreFront(), ()-> _drive.snapHeading(BackHeadingTag.get(getClosestReefTag())));

        get.isPressed(get.driverLeftTrigger() && getClosestReefTag() != -1, ()-> {
            Translation2d tagTranslation = convertWPITranslationToKurtTranslation(kTagLayout.getTagPose(getClosestReefTag()).get().getTranslation().toTranslation2d());
            double tagTheta = kTagLayout.getTagPose(getClosestReefTag()).get().getRotation().getAngle();
            tagTranslation = tagTranslation.plus(new Translation2d(Math.cos(tagTheta)*(poleOffset), Math.sin(tagTheta)*(poleOffset)));
            tagTranslation = tagTranslation.plus(new Translation2d(Math.sin(2*Math.PI - tagTheta)*(robotOffset), Math.cos(2*Math.PI - tagTheta)*(robotOffset)));
            tagTranslation = tagTranslation.plus(adjustment);
            Translation2d robotTranslation = Robot.getPose().getTranslation();
            Translation2d errorTranslation = tagTranslation.minus(robotTranslation);
            yPowerOffset = errorTranslation.getX() * kTagP;
            xPowerOffset = errorTranslation.getY() * kTagP;
            double dist = Robot.getPose().getTranslation().getDistance(tagTranslation);
            if(dist > closeTagDist){
                xPowerOffset = MathUtil.clamp(xPowerOffset,-maxTagPower,maxTagPower);
                yPowerOffset = MathUtil.clamp(yPowerOffset,-maxTagPower,maxTagPower);
            }
            else{
                xPowerOffset = MathUtil.clamp(xPowerOffset,-closeMaxTagPower,closeMaxTagPower);
                yPowerOffset = MathUtil.clamp(yPowerOffset,-closeMaxTagPower,closeMaxTagPower);
            }
        });

        get.isPressed(get.driverRightTrigger() && getClosestReefTag() != -1, ()-> {
            Translation2d tagTranslation = convertWPITranslationToKurtTranslation(kTagLayout.getTagPose(getClosestReefTag()).get().getTranslation().toTranslation2d());
            double tagTheta = kTagLayout.getTagPose(getClosestReefTag()).get().getRotation().getAngle();
            tagTranslation = tagTranslation.plus(new Translation2d(Math.cos(tagTheta)*(-poleOffset), Math.sin(tagTheta)*(-poleOffset)));
            tagTranslation = tagTranslation.plus(new Translation2d(Math.sin(2*Math.PI - tagTheta)*(robotOffset), Math.cos(2*Math.PI - tagTheta)*(robotOffset)));
            tagTranslation = tagTranslation.plus(adjustment);
            Translation2d robotTranslation = Robot.getPose().getTranslation();
            Translation2d errorTranslation = tagTranslation.minus(robotTranslation);
            yPowerOffset = errorTranslation.getX() * kTagP;
            xPowerOffset = errorTranslation.getY() * kTagP;
            double dist = Robot.getPose().getTranslation().getDistance(tagTranslation);
            if(dist > closeTagDist){
                xPowerOffset = MathUtil.clamp(xPowerOffset,-maxTagPower,maxTagPower);
                yPowerOffset = MathUtil.clamp(yPowerOffset,-maxTagPower,maxTagPower);
            }
            else{
                xPowerOffset = MathUtil.clamp(xPowerOffset,-closeMaxTagPower,closeMaxTagPower);
                yPowerOffset = MathUtil.clamp(yPowerOffset,-closeMaxTagPower,closeMaxTagPower);
            }
        });

        get.isNotPressed(List.of(get.driverLeftTrigger(),get.driverRightTrigger()), ()-> {
            xPowerOffset = 0;
            yPowerOffset = 0;
        });

        get.isPressed(driver.getAButton(),() -> Robot.setPose(new Pose2d(0,0, Rotation2d.fromDegrees(0))));

        get.isPressed(driver.getYButton() && scoreFront(),() -> Robot.setPose(new Pose2d(Robot.getPose().getTranslation(), Rotation2d.fromRadians(FrontHeadingTag.get(getClosestReefTag())))));
        get.isPressed(driver.getYButton() && !scoreFront(),() -> Robot.setPose(new Pose2d(Robot.getPose().getTranslation(), Rotation2d.fromRadians(BackHeadingTag.get(getClosestReefTag())))));

        get.isPressed(get.Up() && up, ()-> {adjustment = adjustment.plus(new Translation2d(0,2)); up = false; System.out.println(adjustment.toString());});
        get.isPressed(get.Down() && down, ()-> {adjustment = adjustment.plus(new Translation2d(0,-2)); down = false;System.out.println(adjustment.toString());});
        get.isPressed(get.Right() && right, ()-> {adjustment = adjustment.plus(new Translation2d(2,0)); right = false;System.out.println(adjustment.toString());});
        get.isPressed(get.Left() && left, ()-> {adjustment = adjustment.plus(new Translation2d(-2,0)); left = false;System.out.println(adjustment.toString());});
        get.isNotPressed(get.Up(), ()-> {up = true;});
        get.isNotPressed(get.Down(), ()-> {down = true;});
        get.isNotPressed(get.Right(), ()-> {right = true;});
        get.isNotPressed(get.Left(), ()-> {left = true;});

        driveX = get.driverX();
        driveY = get.driverY();
        driveZ = get.driverZ();

        //apply deadband
        driveX = Math.abs(driveX) > DriveConstants.kDeadband ? driveX : 0.0;
        driveY = Math.abs(driveY) > DriveConstants.kDeadband ? driveY : 0.0;
        driveZ = Math.abs(driveZ) > DriveConstants.kDeadband ? driveZ : 0.0;

        //apply DriveConstants
        driveX = driveX * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * xT;
        driveY = driveY * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * xT;
        driveZ = driveZ * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * rT;

        if(DriverStation.getAlliance().get() == Alliance.Red){
            driveX = -driveX;
            driveY = -driveY;
        }

        _drive.feedSwerveSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(driveX+xPowerOffset,driveY-yPowerOffset,driveZ), Robot.getGyroscopeRotation2d()));
    }
//     public void Operator(){
//         if(!operator.isConnected()){ return; }

//         get.isPressed(get.Ingest() && !_intake.hasPiece(),()-> _intake.setState(IntakeState.ForwardTrip));
//         get.isPressed(get.Outgest(),()-> _intake.setState(IntakeState.Reversed));
//         get.isPressed(get.Score(),() -> _intake.Score(_structure.getScoringLocation()));
//         get.isPressed(get.Score(),() -> get.driverRumble());
//         get.isNotPressed(get.Score(),() -> get.driverUnRumble());
//         get.isNotPressed(List.of(get.Score(), get.Ingest(), get.BallIntake(), get.Outgest(), _intake.hasPiece()),()-> _intake.setState(IntakeState.Idle));
//   }

//         get.isPressed(get.Climb(), ()-> {_structure.setClimbPetition();_manager.getLedInstance().setPartySequence();});
//         get.isPressed(get.ReallyClimb(), ()-> _structure.setClimbClimbPetition());

//         get.isPressed(get.RacketOne() && get.RacketTwo(),()-> {_structure.RacketShoulder();_manager.getLedInstance().setRatchetSequence();});

//         get.isPressed(get.Stow(), ()-> {_structure.setStowPetition(); _manager.getLedInstance().setNoneSequence();});
//         // get.isPressed(get.SourceGrab(), ()-> _structure.setSourceGrabPetition());
//         get.isPressed(get.GroundGrab(), ()-> {_structure.setGroundGrabPetition(); _manager.getLedInstance().setNoneSequence();});

//         get.isPressed(get.BallGrab1(), ()-> _structure.setBall1Petition());
//         get.isPressed(get.BallGrab2(), ()-> _structure.setBall2Petition());
//         get.isPressed(get.BallGround(), ()-> _structure.setBallGroundGrabPetition());

//         get.isPressed(get.Home(), ()->{_manager.getWristInstance().setState(WristState.Home); _manager.getExtensionInstance().setState(ExtensionState.Home);});

//         get.isPressed(get.L1(), ()-> {_structure.setL1Petition(); _manager.getLedInstance().setNoneSequence();});
//         get.isPressed(get.L2() && scoreFront(), ()-> {_structure.setL2FrontPetition(); _manager.getLedInstance().setNoneSequence();});
//         get.isPressed(get.L2() && !scoreFront(), ()-> {_structure.setL2BackPetition(); _manager.getLedInstance().setNoneSequence();});
//         get.isPressed(get.L3() && scoreFront(), ()-> {_structure.setL3FrontPetition(); _manager.getLedInstance().setNoneSequence();});
//         get.isPressed(get.L3() && !scoreFront(), ()-> {_structure.setL3BackPetition(); _manager.getLedInstance().setNoneSequence();});
//         get.isPressed(get.L4(), ()-> {_structure.setL4Petition(); _manager.getLedInstance().setNoneSequence();});
//         if(_structure.getScoringState() == ScoreState.Ground && _intake.hasPiece()){
//             _structure.setStowPetition();
      

        // get.isPressed(get.Outgest(),()-> _manager.getBackUpInstance().runMotor(0.25));
        // get.isNotPressed(get.Outgest(),()-> _manager.getBackUpInstance().runMotor(0.0));
    // }

    public int getClosestReefTag(){
        List<Integer> aprilTags = List.of(6,7,8,9,10,11,17,18,19,20,21,22);
        double minDist = Double.MAX_VALUE;
        int tagId = -1;

        for(int tag : aprilTags){
            double dist = Robot.getPose().getTranslation().getDistance(convertWPITranslationToKurtTranslation(kTagLayout.getTagPose(tag).get().getTranslation().toTranslation2d()));
            if(dist < minDist){
                minDist = dist;
                tagId = tag;
            }
        }

        return tagId;
    }

    Translation2d convertWPITranslationToKurtTranslation(Translation2d initalTranslation){
        return new Translation2d(Units.metersToInches(8.05 - initalTranslation.getY()), Units.metersToInches(initalTranslation.getX()));
    }

    public boolean scoreFront(){
        int closestReefTag = getClosestReefTag();
        double rotationError = Math.abs(kTagLayout.getTagPose(closestReefTag).get().getRotation().getAngle() - Robot.getPose().getRotation().getRadians());
        rotationError = rotationError > Math.toRadians(180) ? 2*Math.PI - rotationError: rotationError;
        if(rotationError <= (Math.PI/2)){
            return false;
        }
        return true;
    }

    public double getRotationVelocityScalarBasedOnExtensionLength(double length){
        double clampedLength = MathUtil.clamp(length,3,15);
        double percentage = 1 - ((clampedLength-3)/24);
        return percentage;
    }

    public void Dashboard(){ 
        SmartDashboard.putNumber("Closest tag id", getClosestReefTag());
        SmartDashboard.putBoolean("Scoring Front", scoreFront());
    }
}