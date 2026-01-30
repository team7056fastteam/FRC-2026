package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.KurtLogger;
import frc.robot.Common.FastSubsystemBase;

public class Kicker extends FastSubsystemBase {

    public enum KickerStates{Idle, Holding, Firing}

    KickerStates state = KickerStates.Idle;

    SparkMax kickerMotor;
    SparkMaxConfig motorConfig;

    KurtLogger logger;

    public Kicker(){
        kickerMotor = new SparkMax(KickerConstants.KickerMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorConfig.inverted(KickerConstants.ReversedKicker);
        kickerMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }

    @Override
    public void Init(KurtLogger logger) {
        this.logger = logger;
    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'run'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public void dashboard() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'dashboard'");
    }
    
    public static final class KickerConstants{
        //TODO contants
        public static final int KickerMotorID = 0;
        public static final boolean ReversedKicker = false;
    }

}
