package frc.robot.Common;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FastTrajectory {
    double maxVelocity;
    double maxAcceleration;
    ArrayList<Pose2d> poses = new ArrayList<>();
    ArrayList<State> states = new ArrayList<>();

    FastTrapzoidMotionProfile motionProfile;

    private double curTime = 0;

    private State curState = null;

    public FastTrajectory(ArrayList<Pose2d> poses, double maxVelocity, double maxAcceleration) {
        this.poses = poses;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        motionProfile = new FastTrapzoidMotionProfile(getTrajectoryTotalDistance(), maxVelocity, maxAcceleration);

        states.clear();
        CalculateTrajectory();
    }

    public double getTrajectoryTotalDistance(){
        double tempDist = 0;
        for (int i = 1; i < poses.size(); i++){
            Pose2d currentPose = poses.get(i - 1);
            Pose2d nextPose = poses.get(i);
    
            tempDist += KurtMath.DistBetweenPose(currentPose, nextPose);
        }
        return tempDist;
    }

    public void CalculateTrajectory(){
        double currentTime = 0.0;
        double odoDist = 0;

        for (int i = 1; i < poses.size(); i++) {
            Pose2d currentPose = poses.get(i - 1);
            Pose2d nextPose = poses.get(i);
    
            double distance = KurtMath.DistBetweenPose(currentPose, nextPose);
            if (distance == 0) continue; // Skip if there's no distance

            odoDist += distance;
            
            currentTime = motionProfile.getTimeBasedOnDistance(odoDist);
            
            double velocity = motionProfile.getVelocityBasedOnTime(currentTime);

            double angle = Math.atan2((nextPose.getX() - currentPose.getTranslation().getX()), (nextPose.getY() - currentPose.getTranslation().getY()));

            double velocityX = velocity * Math.sin(angle);
            double velocityY = velocity * Math.cos(angle);
            
            states.add(new State(nextPose, velocity, velocityX, velocityY, currentTime));
        }
    }

    public double getTrajectoryTotalTime() {
        if (states.isEmpty()) {
            return 0.0;
        }
        return states.get(states.size() - 1).timeStamp;
    }

    // public double getTimeFromTrajectory(Pose2d pose){
    //     if (states.isEmpty()) {
    //         throw new IllegalStateException("Trajectory has not been calculated.");
    //     }
    //     // Find the closest State to our pose
    //     State state = null;
    //     State closestState = null;
    //     double dist = Double.POSITIVE_INFINITY;

    //     for (int i = 0; i < states.size(); i++) {
    //         state = states.get(i);
    //         double tempDist = pose.getTranslation().getDistance(state.pose.getTranslation());
    //         if(tempDist < dist){
    //             dist = tempDist;
    //             closestState = state;
    //         }
    //     }

    //     return closestState.timeStamp;
    // }

    public double getRemainingProgress(){
        return Math.max(0,getTrajectoryTotalTime() - curTime);
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public void reset(){
        curTime = 0;
    }

    public State advance(double additonal){
        curTime += additonal;
        curState = sampleTrajectory(curTime);
        return curState;
    }

    public State preview(double additonal){
        double tempTime = curTime + additonal;
        return sampleTrajectory(tempTime);
    }

    public State getState(){
        curState = sampleTrajectory(curTime);
        return curState;
    }

    public State getLastState(){
        return sampleTrajectory(getTrajectoryTotalTime());
    }

    public State sampleTrajectory(double time) {
        if (states.isEmpty()) {
            throw new IllegalStateException("Trajectory has not been calculated.");
        }

        if (time < states.get(0).timeStamp) {
            return states.get(0);
        }
        if (time > states.get(states.size() - 1).timeStamp) {
            return states.get(states.size() - 1);
        }

        // Find the segment containing the given time
        State startState = null;
        State endState = null;
        for (int i = 0; i < states.size() - 1; i++) {
            startState = states.get(i);
            endState = states.get(i + 1);
            if (time >= startState.timeStamp && time <= endState.timeStamp) {
                break;
            }
        }

        if (startState == null || endState == null) {
            throw new IllegalArgumentException("Time is out of the bounds of the trajectory.");
        }

        // Linear interpolation
        double t = (time - startState.timeStamp) / (endState.timeStamp - startState.timeStamp);
        double interpolatedVelocity = startState.linearVelocity + t * (endState.linearVelocity - startState.linearVelocity);
        double interpolatedVelocityX = startState.linearVelocityX + t * (endState.linearVelocityX - startState.linearVelocityX);
        double interpolatedVelocityY = startState.linearVelocityY + t * (endState.linearVelocityY - startState.linearVelocityY);
        Pose2d interpolatedPose = interpolatePose(startState.pose, endState.pose, t);

        return new State(interpolatedPose, interpolatedVelocity, interpolatedVelocityX, interpolatedVelocityY, time);
    }   

    private Pose2d interpolatePose(Pose2d startPose, Pose2d endPose, double t) {
        double x = startPose.getX() + t * (endPose.getX() - startPose.getX());
        double y = startPose.getY() + t * (endPose.getY() - startPose.getY());
        double rotation = startPose.getRotation().getRadians() + t * (endPose.getRotation().getRadians() - startPose.getRotation().getRadians());
        return new Pose2d(x, y, new Rotation2d(rotation));
    }


    public static class State {
        public Pose2d pose;
        public double linearVelocity;
        public double linearVelocityX;
        public double linearVelocityY;
        public double timeStamp;

        public State(Pose2d pose, double linearVelocity, double linearVelocityX, double linearVelocityY, double timeStamp) {
            this.pose = pose;
            this.linearVelocity = linearVelocity;
            this.linearVelocityX = linearVelocityX;
            this.linearVelocityY = linearVelocityY;
            this.timeStamp = timeStamp;
        }

        public Pose2d getPose(){
            return pose;
        }

        public double distance(State other){
            return pose.getTranslation().getDistance(other.pose.getTranslation());
        }

        public double distance(Pose2d other){
            return pose.getTranslation().getDistance(other.getTranslation());
        }
    }
}