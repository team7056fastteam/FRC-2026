package frc.robot.Common;

import java.util.function.BooleanSupplier;

public class ControllerButton {
    BooleanSupplier supplier;
    public ControllerButton(boolean conditional){
        this.supplier = (()-> conditional);
    }
    public interface VoidInterface {
        void f();
    }
    public void isPressed(VoidInterface mf){
        if (supplier.getAsBoolean()) {
            mf.f();
        }
    }
    public void isNotPressed(VoidInterface mf){
        if (!supplier.getAsBoolean()) {
            mf.f();
        }
    }
}