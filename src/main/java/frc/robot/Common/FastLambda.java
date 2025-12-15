package frc.robot.Common;

public class FastLambda extends FastCommand{

    public interface VoidInterface {
        void f();
    }

    VoidInterface mF;

    public FastLambda(VoidInterface f) {
        this.mF = f;
    }

    @Override
    public void init() {
        mF.f();
    }

    @Override
    public void run() {}

    @Override
    public Boolean isFinished() {
        return true;
    }

    @Override
    public void end() {}
}
