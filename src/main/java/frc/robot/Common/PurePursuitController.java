package frc.robot.Common;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Robot;

public class PurePursuitController {

    private FastTrajectory mCurrentTrajectory = null;
    private FastTrajectory.State mSetpoint;
	private Pose2d endPose = new Pose2d();
    private Lookahead mSpeedLookahead = null;
    private Pose2d mError = new Pose2d();
    private ChassisSpeeds mOutput = new ChassisSpeeds(0,0,0);

	// Pure Pursuit Constants
	public static final double kPathLookaheadTime = 0.1;
	public static final double kPathMinLookaheadDistance = Units.metersToInches(0.3);
	public static final double kAdaptivePathMinLookaheadDistance = Units.metersToInches(0.15);
	public static final double kAdaptivePathMaxLookaheadDistance = Units.metersToInches(0.61);
	public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;
	public static final double kThisIsTheNormalizedRemainingTimeOfTheTrajectoryWhereWeSwitchToAPIDLoopForBetterEndPositioning = .28;
	public static final double kThisIsTheRemainingTimeOfTheTrajectoryWhereWeSwitchToALockedHeading = 1.75;
	public final static double acceptableThetaError = Math.toRadians(5);
	public final static double acceptablePositionError = 4;
	public final static double minCookSpeed = 0.7;

    public void setTrajectory(FastTrajectory trajectory) {
		mCurrentTrajectory = trajectory;
		mSetpoint = trajectory.getState();
		mSpeedLookahead = new Lookahead(
				kAdaptivePathMinLookaheadDistance,
				kAdaptivePathMaxLookaheadDistance,
				0.0,
				Constants.AutoConstants.kMaxSpeedInchesPerSecond);
		endPose = mCurrentTrajectory.getLastState().pose;
	}

	ChassisSpeeds updatePID(Pose2d current_state){
		final double kThetakP = 4.1;
		final double kPositionkP = 3.4;
		final double maxPosPower = 16;

		System.out.println("here? PID");

		Pose2d mPidError = new Pose2d(endPose.getX() - current_state.getX(), endPose.getY() - current_state.getY(), endPose.getRotation().minus(current_state.getRotation()));

		double xFeedback = kPositionkP * mPidError.getTranslation().getX();
		double yFeedback = kPositionkP * mPidError.getTranslation().getY();
		double zFeedback = kThetakP * mPidError.getRotation().getRadians();

		xFeedback = MathUtil.clamp(xFeedback,-maxPosPower,maxPosPower);
        yFeedback = MathUtil.clamp(yFeedback,-maxPosPower,maxPosPower);

		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
			Units.inchesToMeters(yFeedback), 
			-1*Units.inchesToMeters(xFeedback),zFeedback
		),Robot.getGyroscopeRotation2d());

		return chassisSpeeds;
	}

    ChassisSpeeds updatePurePursuit(Pose2d current_state){
		System.out.println("here? Pure");
        double lookahead_time = kPathLookaheadTime;
		final double kLookaheadSearchDt = 0.01;
		FastTrajectory.State lookahead_state =
				mCurrentTrajectory.preview(lookahead_time);
		double actual_lookahead_distance = mSetpoint.distance(lookahead_state);
		double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mSetpoint.linearVelocity)
				+ kAdaptiveErrorLookaheadCoefficient * mError.getTranslation().getNorm();
		// Find the Point on the Trajectory that is Lookahead Distance Away
		while (
			actual_lookahead_distance < adaptive_lookahead_distance && mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
			lookahead_time += kLookaheadSearchDt;
			lookahead_state = mCurrentTrajectory.preview(lookahead_time);
			actual_lookahead_distance = mSetpoint.distance(lookahead_state);
		}

		// If the Lookahead Point's Distance is less than the Lookahead Distance, transform it so it is the lookahead
		// distance away
		if (actual_lookahead_distance < adaptive_lookahead_distance) {
			Pose2d transformedPose = lookahead_state.getPose();
			transformedPose = transformBy(transformedPose,transformedPose.getTranslation().plus(new Translation2d((kPathMinLookaheadDistance - actual_lookahead_distance),0.0)));
			lookahead_state = new FastTrajectory.State(transformedPose, lookahead_state.linearVelocity, lookahead_state.linearVelocityX, lookahead_state.linearVelocityY, lookahead_state.timeStamp);
		}

		//System.out.println(lookahead_time);

		if (lookahead_state.linearVelocity == 0.0) {
			mCurrentTrajectory.advance(Double.POSITIVE_INFINITY);
			return new ChassisSpeeds();
		}

		// SmartDashboard.putNumber("RobotX", mSetpoint.getPose().getX());
		// SmartDashboard.putNumber("RobotY", mSetpoint.getPose().getY());

		// SmartDashboard.putNumber("RobotX", current_state.getX());
		// SmartDashboard.putNumber("RobotY", current_state.getY());

		// SmartDashboard.putNumber("NavPodX", lookahead_state.getPose().getX());
		// SmartDashboard.putNumber("NavPodY", lookahead_state.getPose().getY());

		// Use the Velocity Feedforward of the Closest Point on the Trajectory
		double normalizedSpeed = Math.abs(mSetpoint.linearVelocity) / Constants.AutoConstants.kMaxSpeedInchesPerSecond;

		if(normalizedSpeed < minCookSpeed && lookahead_state.timeStamp < mCurrentTrajectory.getTrajectoryTotalTime()/2.0){
			normalizedSpeed = minCookSpeed;
		}

		double angle = Math.atan2((lookahead_state.getPose().getX() - current_state.getTranslation().getX()), (lookahead_state.getPose().getY() - current_state.getTranslation().getY()));

        double velocityX = normalizedSpeed * Math.sin(angle);
        double velocityY = normalizedSpeed * Math.cos(angle);

		// // Use the PD-Controller for To Follow the Time-Parametrized Heading
		final double kThetakP = 3.9;
		// //final double kThetakD = 0.0;
		final double kPositionkP = 2;

		double xFeedback = kPositionkP * mError.getTranslation().getX();
		double yFeedback = kPositionkP * mError.getTranslation().getX();
		double zFeedback = kThetakP * mError.getRotation().getRadians();

		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
			Units.inchesToMeters(velocityY * Constants.AutoConstants.kMaxSpeedInchesPerSecond + yFeedback), 
			-1*Units.inchesToMeters(velocityX * Constants.AutoConstants.kMaxSpeedInchesPerSecond + xFeedback),zFeedback
		),Robot.getGyroscopeRotation2d());

		return chassisSpeeds;
    }

    public ChassisSpeeds update(Pose2d current_state){
        if (mCurrentTrajectory == null) return new ChassisSpeeds();

		FastTrajectory.State sample_point;

        if(!isDone()){
            //mPrevHeadingError = mError.getRotation();
			mError = new Pose2d(mSetpoint.getPose().getX() - current_state.getX(), mSetpoint.getPose().getY() - current_state.getY(), mSetpoint.getPose().getRotation().minus(current_state.getRotation()));

			// if(mCurrentTrajectory.getRemainingProgress() < kThisIsTheRemainingTimeOfTheTrajectoryWhereWeSwitchToALockedHeading){
			// 	mError = new Pose2d(mError.getTranslation(), endPose.getRotation().minus(current_state.getRotation()));
			// }

            double searchStepSize = 3.5;
			double previewQuantity = 0.0;
			double searchDirection = 1.0;
			double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
			double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
			searchDirection = Math.signum(reverseDistance - forwardDistance);
			while (searchStepSize > 0.001) {
				if (epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.0003937)) break;
				while (
				/* next point is closer than current point */ distance(
								current_state, previewQuantity + searchStepSize * searchDirection)
						< distance(current_state, previewQuantity)) {
					/* move to next point */
					previewQuantity += searchStepSize * searchDirection;
				}
				searchStepSize /= 10.0;
				searchDirection *= -1;
			}
			sample_point = mCurrentTrajectory.advance(previewQuantity);
			mSetpoint = sample_point;
			System.out.println(mCurrentTrajectory.getRemainingProgress());
			// if(mCurrentTrajectory.getTrajectoryTotalTime() != 0){
			if((mCurrentTrajectory.getRemainingProgress()/mCurrentTrajectory.getTrajectoryTotalTime()) > kThisIsTheNormalizedRemainingTimeOfTheTrajectoryWhereWeSwitchToAPIDLoopForBetterEndPositioning){
				mOutput = updatePurePursuit(current_state);
			}
			else if(mCurrentTrajectory.getTrajectoryTotalTime() != 0){
				mOutput = updatePID(current_state);
			// 	// mOutput = new ChassisSpeeds();
			}
			else{
				mOutput = new ChassisSpeeds();
				System.out.println("here");
			}
			
        }
        else{
            if (mCurrentTrajectory.getLastState().linearVelocity == 0.0) {
				mOutput = new ChassisSpeeds();
			}
        }

        return mOutput;
    }

    public boolean isDone() {
		return mCurrentTrajectory != null && atReferenceAndHeading(Robot.getPose(), endPose);
	}

    public Pose2d inversePose(Pose2d pose) {
        Rotation2d rotation_inverted = Rotation2d.fromRadians(-pose.getRotation().getRadians());
        Translation2d translation_inverted = new Translation2d(-pose.getX(), -pose.getY());
        return new Pose2d(translation_inverted.rotateBy(rotation_inverted), rotation_inverted);
    }

    private double distance(Pose2d current_state, double additional_progress) {
		return mCurrentTrajectory.preview(additional_progress).distance(current_state);
	}

    public static boolean epsilonEquals(double a, double b, double epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}

	public boolean atReferenceAndHeading(Pose2d robotPose, Pose2d desiredPose){
        double xError = Math.abs(robotPose.getX() - desiredPose.getX());
        double yError = Math.abs(robotPose.getY() - desiredPose.getY());
        double thetaError = Math.abs(robotPose.getRotation().getRadians() - desiredPose.getRotation().getRadians());
        if(thetaError > Math.PI){
            thetaError = Math.abs(2*Math.PI - thetaError);
        }

        if(xError < acceptablePositionError && yError < acceptablePositionError && thetaError < acceptableThetaError){
            return true;
        }
        return false;
    }

	public Pose2d transformBy(Pose2d pose, Translation2d other) {
    return new Pose2d(
        pose.getTranslation().plus(other.rotateBy(pose.getRotation())),
        pose.getRotation());
  }
}
