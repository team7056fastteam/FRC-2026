package frc.robot.Common;

import java.util.List;

public class ControllerButtonGroup {
    List<ControllerButton> buttons;
    public ControllerButtonGroup(List<ControllerButton> buttons){
        this.buttons = buttons;
    }
    public interface VoidInterface {
        void f();
    }
    public void isNotPressed(VoidInterface mf){
        for (ControllerButton button : buttons){
            if(!button.supplier.getAsBoolean()){
                return;
            }
            mf.f();
        }
    }
}
