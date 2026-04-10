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
    public boolean autoOrient(){
        return driver.getRawAxis(2) > 0.1;
    }
    /**Driver Right Trigger*/
    public boolean speedAdjustment(){
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
    /**Driver Y Button*/
    public boolean alignFront(){
        return driver.getYButton();
    }
    /**Driver B Button*/
    public boolean alignRight(){
        return driver.getBButton();
    }
    /**Driver A Button*/
    public boolean alignBack(){
        return driver.getAButton();
    }
    /**Driver X Button*/
    public boolean alignLeft(){
        return driver.getXButton();
    }
    /**Driver Left Bumper*/
    public boolean LeftBumper(){
        return driver.getLeftBumperButton();
    }
    /**Driver Right Bumper */
    public boolean rotateBump(){
        return driver.getRightBumperButton();
    }
    /**Driver D-Pad Any*/
    public boolean Reset(){
        return Up() || Down() || Right() || Left();
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
        return driver.getLeftBumperButton();
    }
    /**Operator A Button*/
    public boolean RevShooter(){
        return operator.getAButton();
    }
    /**Operator B Button*/
    public boolean Shoot(){
        return operator.getBButton();
    }
    /**Operator Y Button Pressed */
    public boolean SpindexerSlow(){
        return operator.getYButton();
    }
    /**Operator X Button*/
    public boolean Pass(){
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
    public boolean IngestSlow(){
        return operator.getRawAxis(2) > 0.1;
    }
    /**Operator Right Trigger*/
    public boolean UnstuckalateKicker(){
        return operator.getRawAxis(3) > 0.1;
    }
    /**Operator POV Up*/
    public boolean OverrideShortShot(){
        return operator.getPOV() == 0;
    }
    /**Operator POV Down*/
    public boolean OverrideLongShot(){
        return operator.getPOV() == 180;
    }
    /**Operator POV Right*/
    public boolean OverrideMidShot(){
        return operator.getPOV() == 90;
    }
    /**Operator POV Left*/
    public boolean AutoTargeting(){
        return operator.getPOV() == 270;
    }
    /**Operator Left Stick Up*/
    public boolean FreeFire(){
        return operator.getRawAxis(1) < -0.5;
    }
    /**Operator Left Stick Down*/
    public boolean HoldMode(){
        return operator.getRawAxis(1) > 0.5;
    }
    /**Operator Right Stick Up*/
    public boolean OverrideAtSpeed(){
        return operator.getRawAxis(5) < -0.5;
    }
    /**Operator Right Stick Down*/
    public boolean ClimbDown(){
        return operator.getRawAxis(5) > 0.5;
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
