package frc.robot.Common;

public class AutoModeRunner {
    private FastAutoBase mAuto;
    private Thread m_thread = null;

    public void setAuto(FastAutoBase mAuto) {
        this.mAuto = mAuto;
    }

    public void start() {
        if (m_thread == null) {
            m_thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    if (mAuto != null) {
                        mAuto.run();
                    }
                }
            });
            m_thread.start();
        }
    }

    public void stop() {
        if (mAuto != null){
            mAuto.stop();
        }
        m_thread = null;
    }
}
