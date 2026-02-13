package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ModuleConstants {
        //wheel diameter in inches
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

        //look these numbers up
        public static final double kDriveMotorGearRatio = 1 / 5.14;
        public static final double kTurningMotorGearRatio = 1 / 18.0;

        //these convert thoose numbers into positon and velocity
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    }
    public static final class DriveConstants {
        //measure left to right wheel
        public static final double kTrackWidth = Units.inchesToMeters(19.5);

        //measure front to back wheel
        public static final double kWheelBase = Units.inchesToMeters(21.5);

        //should be same if you have a square robot which is typically what you want

        public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
            new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
            new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
            new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0));

        //drive motors port ids
        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kBackLeftDriveMotorPort = 7;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 5;
        
        //turn motors port ids
        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kBackLeftTurningMotorPort = 8;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackRightTurningMotorPort = 6;

        //if the wheels are turning forever they flip the corresponding value
        public static final boolean kFrontLeftTurningMotorReversed = false;
        public static final boolean kBackLeftTurningMotorReversed = false;
        public static final boolean kFrontRightTurningMotorReversed = false;
        public static final boolean kBackRightTurningMotorReversed = false;

        //to test this put robot up so wheels aren't touching ground and if you put the stick all the way forward they should all be driving in the forward direction
        //if not they adjust this value
        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kBackLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = true;
        public static final boolean kBackRightDriveMotorReversed = false;

        //abs encoders ids
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 4;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        //don't think this needs to be adjusted
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //adjust wheel offsets
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(0.0);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(0.0);

        //these are the physical max of the motor. Look up the values for these.
        public static final double kPhysicalMaxSpeedMetersPerSecond = 6.03504;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        //adjust the divisor closer to 1 but never past if you want more speed
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.85;

        //adjust the divisor closer to 1 but never past if you want faster turning
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;

        //adjust these values for faster acceleration during teleOp
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.5;

        //adjust this value if your robot is moving without you touching the sticks. the older controller the more this number typically is
        //you probally want to replace controllers after two seasons or if the stick drift is too high for preicous movement of robot
        public static final double kDeadband = 0.09;
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 1.9;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 8;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.07; //0.4
        public static final double kPYController = 0.07; //0.4
        public static final double kIXController = 0.000; //0.0125
        public static final double kIYController = 0.000; //0.0125
        
        public static final double kPThetaController1 = 7;
        public static final double kPThetaController = 2; //3
        public static final double kPThetaController0 = 2;
        public static final double kPTargetController = 0.075;
        public static final double diagonalController = 0.0125;

        public static final double kMaxSpeedInchesPerSecond = Units.metersToInches(kMaxSpeedMetersPerSecond);
        public static final double kMaxAccelerationInchesPerSecondSquared = Units.metersToInches(kMaxAccelerationMetersPerSecondSquared);
    }

    public static final class FieldConstants {
        public static final Translation2d hubPosBlue = new Translation2d(4.625,4.03);
        public static final Translation2d hubPosRed = new Translation2d(11.915, 4.03);
        public static final double hubHeight = 1.83;
    }

}
