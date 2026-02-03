package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.KurtLogger;
import frc.robot.Common.FastSubsystemBase;

public class Kicker extends FastSubsystemBase {
    public enum KickerState{Idle, Holding, Firing}

    KickerState state = KickerState.Idle;
    CANrange fuelSensor;
    SparkMax kickerMotor;
    SparkMaxConfig motorConfig;

    KurtLogger logger;

    public Kicker(){
        kickerMotor = new SparkMax(KickerConstants.KickerMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        fuelSensor = new CANrange(KickerConstants.FuelSensorID);
        motorConfig.inverted(KickerConstants.ReversedKicker);
        kickerMotor.configure(motorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }

    @Override
    public void Init(KurtLogger logger) {
        this.logger = logger;
    }

    @Override
    public void run() {
        switch (state) {
            case Firing:
                kickerMotor.set(KickerConstants.KickerFiringVelocity);
                break;
            case Holding:
                if(fuelSensor.getDistance().getValueAsDouble() < Units.inchesToMeters(2)){
                    kickerMotor.stopMotor();
                    state = KickerState.Idle;
                } else {
                    kickerMotor.set(KickerConstants.KickerHoldingVelocity);
                }
                break;
            case Idle:
                kickerMotor.stopMotor();
                break;
        }
    }

    @Override
    public void stop() {
        kickerMotor.stopMotor();
    }

    @Override
    public void dashboard() {
        SmartDashboard.putNumber("Kicker Current", kickerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Kicker RPM", kickerMotor.getEncoder().getVelocity());
        SmartDashboard.putString("Kicker State", state.toString());
    }

    public void setState(KickerState state){
        this.state = state;
    }
    
    public static final class KickerConstants{
        //TODO contants
        public static final int KickerMotorID = 0;
        public static final boolean ReversedKicker = false;
        public static final double KickerFiringVelocity = 0;
        public static final int FuelSensorID = 0;
        public static final double KickerHoldingVelocity = .1;
    }

}
