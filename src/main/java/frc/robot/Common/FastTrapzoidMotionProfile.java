package frc.robot.Common;

public class FastTrapzoidMotionProfile {
    //Input Variables
    double totalDistance; //dt or total distance
    double maxVelocity;
    double maxAcceleration;

    //Helper Variables
    double t0; //end of the first slope
    double t1; //end of constant slope
    double t2; //end of curve

    double d0; //length from beginning to t0
    double d1; //length from beginning to t1

    double maxPossibleVelocity; // max velocity isn't reachable on any given distance

    // v(t)   _____________________________
    //       /|                           |\
    //      / |                           | \
    //     /  |                           |  \
    //    / t0|                         t1|   \t2
    //        d0                          d1  dt

    public FastTrapzoidMotionProfile(double totalDistance, double maxVelocity, double maxAcceleration){
        this.totalDistance = totalDistance;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        d0 = (.5)*((maxVelocity*maxVelocity)/maxAcceleration);

        if(totalDistance - (2*d0) <= 0){
            //Handle the minimum distance not being met
            d0 = (totalDistance/3);
            d1 = (totalDistance/3) + d0;

            maxPossibleVelocity = Math.sqrt(2 * maxAcceleration * d0);

            t0 = Math.sqrt((2*d0)/maxAcceleration);
            t1 = ((d1-d0)/maxPossibleVelocity) + t0;
            t2 = t1 + t0;
        }
        else{
            maxPossibleVelocity = maxVelocity;

            d1 = totalDistance - d0;

            t0 = Math.sqrt((2*d0)/maxAcceleration);
            t1 = ((d1-d0)/maxPossibleVelocity) + t0;
            t2 = t1 + t0;
        }
    }

    public double getTimeBasedOnDistance(double dist){
        if(dist <= d0){
            //phase 1
            return Math.sqrt((2*dist)/maxAcceleration);
        }
        else if((dist < d1) && (t0 != t1)){
            //phase 2
            return ((dist - d0) / maxPossibleVelocity) + t0;
        }
        else if(dist < totalDistance){
            //phase 3
            return (t2 - Math.sqrt((2*(totalDistance-dist))/maxAcceleration));
        }
        else{
            return t2;
        }
    }
    public double getVelocityBasedOnTime(double time){
        if(time <= t0){
            //phase 1
            return maxAcceleration * time;
        }
        else if((time < t1) && (t0 != t1)){
            //phase 2
            return maxPossibleVelocity;
        }
        else if(time < t2){
            //phase 3
            return (maxAcceleration*(t2 - time));
        }
        else{
            return 0;
        }
    }
}
