package frc.robot.Common;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ControllerFunction {
    XboxController driver, operator;

    public ControllerFunction(XboxController driver, XboxController operator){
        this.driver = driver;
        this.operator = operator;
    }

    public double driverX(){
        return driver.getRawAxis(1) * -1;
    }

    public double driverY(){
        return driver.getRawAxis(0) * -1;
    }

    public double driverZ(){
        return driver.getRawAxis(4) * -1;
    }
    /**Driver Left Trigger*/
    public boolean driverLeftTrigger(){
        return driver.getRawAxis(2) > 0.1;
    }
    /**Driver Right Trigger*/
    public boolean driverRightTrigger(){
        return driver.getRawAxis(3) > 0.1;
    }

    public interface VoidInterface {
        void f();
    }

    public void isPressed(Boolean conditional, VoidInterface mf){
        if (conditional) {
            mf.f();
        }
    }

    public void isNotPressed(Boolean conditional, VoidInterface mf){
        if (!conditional) {
            mf.f();
        }
    }

    public void isNotPressed(List<Boolean> conditionals, VoidInterface mf){
        for (Boolean button : conditionals){
            if(button){
                return;
            }
        }
        mf.f();
    }
    /**Driver Left Bumper*/
    public boolean speedAdjustment(){
        return driver.getLeftBumperButton();
    }
    /**Driver A Button*/
    public boolean Reset(){
        return driver.getAButton();
    }
    /**Driver Y Button*/
    public boolean ResetHeadingToNearestTag(){
        return driver.getYButton();
    }
    /**Driver D-Pad Up */
    public boolean Up(){
        return driver.getPOV() == 0;
    }
    /**Driver D-Pad Down */
    public boolean Down(){
        return driver.getPOV() == 180;
    }
    /**Driver D-Pad Right */
    public boolean Right(){
        return driver.getPOV() == 90;
    }
    /**Driver D-Pad Left */
    public boolean Left(){
        return driver.getPOV() == 270;
    }
    /**Driver Left Trigger*/
    public boolean lockWheels(){
        return driver.getRawAxis(2) > 0.1;
    }
    /**Operator A Button*/
    public boolean Stow(){
        return operator.getAButton();
    }
    /**Operator B Button*/
    public boolean Score(){
        return operator.getBButton();
    }
    /**Operator Y Button*/
    public boolean GroundGrab(){
        return operator.getYButton();
    }
    /**Operator X Button*/
    public boolean Home(){
        return operator.getXButton();
    }
    /**Operator Left Bumper*/
    public boolean Ingest(){
        return operator.getLeftBumperButton();
    }
    /**Operator Right Bumper*/
    public boolean Outgest(){
        return operator.getRightBumperButton();
    }
    /**Operator Left Trigger*/
    public boolean FlipWrist(){
        return operator.getRawAxis(2) > 0.1;
    }
    /**Operator Left Trigger*/
    public boolean RacketOne(){
        return operator.getRawAxis(2) > 0.1;
    }
    /**Operator Right Trigger*/
    public boolean RacketTwo(){
        return operator.getRawAxis(3) > 0.1;
    }
    // /**Operator POV Up*/
    public boolean L4(){
        return operator.getPOV() == 0;
    }
    /**Operator POV Down*/
    public boolean L2(){
        return operator.getPOV() == 180;
    }
    /**Operator POV Right*/
    public boolean L3(){
        return operator.getPOV() == 90;
    }
    /**Operator POV Left*/
    public boolean L1(){
        return operator.getPOV() == 270;
    }
    /**Operator Stick Up*/
    public boolean BallGrab2(){
        return operator.getRawAxis(1) < -0.5;
    }
    /**Operator Stick Down*/
    public boolean BallGrab1(){
        return operator.getRawAxis(1) > 0.5;
    }
    /**Operator Stick Right*/
    public boolean BallGround(){
        return operator.getRawAxis(0) > 0.5;
    }
    /**Operator Right Trigger */
    public boolean BallIntake(){
        return operator.getRightStickButton();
    }
    /**Operator Right Stick Up*/
    public boolean Climb(){
        return operator.getRawAxis(5) < -0.1;
    }

    /**Operator Right Stick Down*/
    public boolean ReallyClimb(){
        return operator.getRawAxis(5) > 0.1;
    }

    public void driverRumble(){
        driver.setRumble(RumbleType.kBothRumble, 0.25);
    }

    public void driverUnRumble(){
        driver.setRumble(RumbleType.kBothRumble, 0);
    }

    public void operatorRumble(){
        operator.setRumble(RumbleType.kBothRumble, 1);
    }

    public void Button(boolean active, FastTeleOpAction action){
        if(active){
            action.run();
        }
    }
}
