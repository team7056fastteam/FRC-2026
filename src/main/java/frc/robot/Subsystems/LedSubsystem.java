package frc.robot.Subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Robot;
import frc.robot.Odometry;

public class LedSubsystem extends SubsystemBase {
    private static PWM redPWM;
    private static PWM greenPWM;
    private static PWM bluePWM;

    private LedSequence activeSequence = null;
    private int sequenceIndex = 0;
    private double timerOffset = 0;

    private final Odometry _odometry;

    public LedSubsystem() {
        redPWM = new PWM(LedConstants.redPin);
        greenPWM = new PWM(LedConstants.greenPin);
        bluePWM = new PWM(LedConstants.bluePin);

        timerOffset = Timer.getFPGATimestamp();
        sequenceIndex = 0;
        activeSequence = null;

        _odometry = Robot.getOdometryInstance();
    }

    @Override
    public void periodic() {
        // Update LEDs based on hub distance & field side
        updateRangeLEDs();

        // Animate LED sequence
        if (activeSequence != null) {
            if (Timer.getFPGATimestamp() - timerOffset > activeSequence.delay) {
                timerOffset = Timer.getFPGATimestamp();
                if (sequenceIndex >= activeSequence.colors.size()) {
                    sequenceIndex = 0;
                }
                Color activeColor = activeSequence.colors.get(sequenceIndex);
                setRGB(activeColor.getR(), activeColor.getG(), activeColor.getB());
                sequenceIndex++;
            }
        } else {
            sequenceIndex = 0;
            setRGB(0, 0, 0);
        }
    }

    public void stop() {
        activeSequence = null;
    }

    public void dashboard() {
        SmartDashboard.putString("Led Status", activeSequence != null ? activeSequence.toString() : "None");
    }

    public static void setRGB(int r, int g, int b) {
        r = (int) ((r / 255.0) * 4095);
        g = (int) ((g / 255.0) * 4095);
        b = (int) ((b / 255.0) * 4095);
        redPWM.setPulseTimeMicroseconds(r);
        greenPWM.setPulseTimeMicroseconds(g);
        bluePWM.setPulseTimeMicroseconds(b);
    }

    public void setShortRangeSequence() {
        activeSequence = new LedSequence(List.of(Color.Green, Color.None), 0.075);
    }

    public void setMidRangeSequence() {
        activeSequence = new LedSequence(List.of(Color.Blue, Color.None), 0.075);
    }

    public void setFarRangeSequence() {
        activeSequence = new LedSequence(List.of(Color.Purple, Color.None), 0.075);
    }

    public void setOutOfRangeSequence() {
        activeSequence = new LedSequence(List.of(Color.Red, Color.None), 0.075);
    }

    public void setPartySequence() {
        activeSequence = new LedSequence(List.of(Color.Red, Color.Green, Color.Blue, Color.None), 0.2);
    }

    public void setNoneSequence() {
        activeSequence = null;
    }

    // Update LEDs based on distance and field side
    public void updateRangeLEDs() {
        Pose2d robotPose = _odometry.getPose();
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        double hubX = (alliance == Alliance.Blue ? FieldConstants.hubPosBlue.getX() : FieldConstants.hubPosRed.getX());
        double hubY = (alliance == Alliance.Blue ? FieldConstants.hubPosBlue.getY() : FieldConstants.hubPosRed.getY());

        double dx = robotPose.getX() - hubX;
        double dy = robotPose.getY() - hubY;

        // Only light LEDs if robot is on the correct side of the hub
        boolean onCorrectSide = (alliance == Alliance.Blue) ? (dx < 0) : (dx > 0);
        if (!onCorrectSide) {
            setNoneSequence();
            return;
        }

        double distance = Math.sqrt(dx * dx + dy * dy);
        double closeThreshold = Units.inchesToMeters(35.25);
        double midLower = Units.inchesToMeters(108);
        double midUpper = Units.inchesToMeters(125);
        double midPoint = (midLower + midUpper) / 2.0;

        if (distance < closeThreshold) {
            setOutOfRangeSequence();
        } else if (distance >= closeThreshold && distance < midPoint) {
            setMidRangeSequence();
        } else if (distance >= midPoint && distance < midUpper) {
            setFarRangeSequence();
        } else {
            setNoneSequence();
        }
    }
}

class Color {
    private final int r, g, b;

    public static final Color Purple = new Color(255, 0, 255);
    public static final Color White = new Color(255, 255, 255);
    public static final Color Green = new Color(0, 255, 0);
    public static final Color Red = new Color(255, 0, 0);
    public static final Color Blue = new Color(0, 0, 255);
    public static final Color None = new Color(0, 0, 0);

    public Color(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public int getR() { return r; }
    public int getG() { return g; }
    public int getB() { return b; }
}

class LedSequence {
    List<Color> colors;
    double delay;

    public LedSequence(List<Color> colors, double delay) {
        this.colors = colors;
        this.delay = delay;
    }
}

class LedConstants {
    //TODO led constants
    public static final int redPin = 2;
    public static final int greenPin = 1;
    public static final int bluePin = 3;
}
