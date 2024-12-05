package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Swerve;

public class Constants {

    public class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.2)
                .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(3).withKI(0).withKD(0)
                .withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 150.0;

        // Initial configs for the drive and steer motors and the CANcoder; these cannot
        // be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API
        // documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                // Swerve azimuth does not require much torque output, so we can set a
                                // relatively low
                                // stator current limit to help avoid brownouts without impacting performance.
                                .withStatorCurrentLimit(60)
                                .withStatorCurrentLimitEnable(true));
        private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 10.43;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.122448979591837;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final double kWheelRadiusInches = 4;

        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final String kCANbusName = "Canivore";
        private static final int kPigeonId = 23;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

        private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANbusName(kCANbusName)
                .withPigeon2Id(kPigeonId)
                .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withWheelRadius(kWheelRadiusInches)
                .withSlipCurrent(kSlipCurrentA)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                .withCouplingGearRatio(kCoupleRatio)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withCANcoderInitialConfigs(cancoderInitialConfigs);

        // Front Left
        private static final int kFrontLeftDriveMotorId = 1;
        private static final int kFrontLeftSteerMotorId = 2;
        private static final int kFrontLeftEncoderId = 31;
        private static final double kFrontLeftEncoderOffset = -0.23388671875;
        private static final boolean kFrontLeftSteerInvert = true;

        private static final double kFrontLeftXPosInches = 12.5;
        private static final double kFrontLeftYPosInches = 12.5;

        // Front Right
        private static final int kFrontRightDriveMotorId = 3;
        private static final int kFrontRightSteerMotorId = 4;
        private static final int kFrontRightEncoderId = 32;
        private static final double kFrontRightEncoderOffset = 0.382568359375;
        private static final boolean kFrontRightSteerInvert = true;

        private static final double kFrontRightXPosInches = 12.5;
        private static final double kFrontRightYPosInches = -12.5;

        // Back Left
        private static final int kBackLeftDriveMotorId = 5;
        private static final int kBackLeftSteerMotorId = 6;
        private static final int kBackLeftEncoderId = 33;
        private static final double kBackLeftEncoderOffset = 0.128173828125;
        private static final boolean kBackLeftSteerInvert = true;

        private static final double kBackLeftXPosInches = -12.5;
        private static final double kBackLeftYPosInches = 12.5;

        // Back Right
        private static final int kBackRightDriveMotorId = 7;
        private static final int kBackRightSteerMotorId = 8;
        private static final int kBackRightEncoderId = 34;
        private static final double kBackRightEncoderOffset = 0.004150390625;
        private static final boolean kBackRightSteerInvert = true;

        private static final double kBackRightXPosInches = -12.5;
        private static final double kBackRightYPosInches = -12.5;

        private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
        private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches),
                kInvertRightSide);
        private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
        private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
                kInvertRightSide);

        public static final Swerve DriveTrain = new Swerve(DrivetrainConstants,
                FrontLeft,
                FrontRight, BackLeft, BackRight);
    }

    public class DrivetrainConstants {

        public static final double MaxSpeed = Units.feetToMeters(18);
        private static final double WHEELBASE = TunerConstants.kFrontLeftXPosInches * 2; // 2 * x distance from center
                                                                                         // of robot to wheel
        public static final double MaxAngularRate = 2 * Math.PI
                * (TunerConstants.kSpeedAt12VoltsMps / Math.PI * Math.sqrt(2 * Math.pow(WHEELBASE, 2)));

        public static final double ROT_MULT = 0.04;

        public static final double NORMAL_ROT_MULT = 1;
        public static final double NORMAL_SPEED_MULT = 1;

        public static final double SLOW_ROT_MULT = 0.7;
        public static final double SLOW_SPEED_MULT = 0.4;

        public static final double SYS_TEST_SPEED_DRIVE = 0.5;
        public static final double SYS_TEST_SPEED_TURN = 0.7d;

        public static final double ALIGNMENT_TOLERANCE = 1d;
    }

    public class PivotConstants {
        public static final boolean MOTOR_RIGHT_INVERT = true; // POS power is up
        public static final boolean MOTOR_LEFT_INVERT = true; // POS power is up
        public static final int MOTOR_STATOR_CURRENT_LIMIT = 60;
        public static final boolean MOTOR_BRAKE_MODE = true;
        public static final double MOTOR_KP = 0;
        public static final double MOTOR_KI = 0;
        public static final double MOTOR_KD = 0;
        public static final double MOTOR_KS = 0;
        public static final double MOTOR_KV = 3;
        public static final double MOTOR_KA = 0;

        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 0;

        // Not currently using Motion magic
        public static final double MAGIC_CRUISE_VEL = 0.01;
        public static final double MAGIC_ACCEL = 0.02;
        public static final double MAGIC_JERK = 0.2;

        public static final double ANGLE_TOLERANCE = 0.00208d;

        public static final double ENCODER_OFFSET = 0.61095; // In rotations
        public static final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
        public static final double ENCODER_TO_MECHANISM_RATIO = 1d;
        public static final double ROTOR_TO_ENCODER_RATIO = 618.75;
    }

    public class FlywheelConstants {
        public static final boolean LEFT_INVERT = false;
        public static final boolean RIGHT_INVERT = true;

        public static final int MOTOR_STATOR_CURRENT_LIMIT = 80;

        public static final double MOTOR_KP = 0.2;
        public static final double MOTOR_KI = 0.07;
        public static final double MOTOR_KD = 0;
        public static final double MOTOR_KS = 0.26;
        public static final double MOTOR_KV = 0.11;
        public static final double MOTOR_KA = 2.9;
    }

    public class RobotMap {
        public class CAN {
            // Front Left
            private static final int kFrontLeftDriveMotorId = 1;
            private static final int kFrontLeftSteerMotorId = 2;
            private static final int kFrontLeftEncoderId = 31;

            // Front Right
            private static final int kFrontRightDriveMotorId = 3;
            private static final int kFrontRightSteerMotorId = 4;
            private static final int kFrontRightEncoderId = 32;

            // Back Left
            private static final int kBackLeftDriveMotorId = 5;
            private static final int kBackLeftSteerMotorId = 6;
            private static final int kBackLeftEncoderId = 33;

            // Back Right
            private static final int kBackRightDriveMotorId = 7;
            private static final int kBackRightSteerMotorId = 8;
            private static final int kBackRightEncoderId = 34;

            public static final int PigeonId = 23;

            public static final int PIVOT_LEFT_ID = 9;
            public static final int PIVOT_RIGHT_ID = 10;
            public static final int PIVOT_CANCODER_ID = 35;

            public static final int INDEXER_ID = 11;
            public static final int SHOOTER_LEFT_ID = 12;
            public static final int SHOOTER_RIGHT_ID = 13;

            public static final String CANBUS_FD = "Canivore";
        }
        
        public class VisionConstants {
            public static final String camera1Name = "2311_Cam1";
        }
    }

    public class IndexerConstants {

        public static final boolean INDEXER_INVERT = false;
        public static final double MOTOR_STATOR_CURRENT_LIMIT = 60;

        public static final double DEFAULT_INDEXER_POWER = 0.5;
    }

    public class AutonomousConstants {

    }

    public class PoseConstants {
        public static final Translation2d FIELD_LIMIT = new Translation2d(Units.feetToMeters(54.0), Units.feetToMeters(26.0));
    }
}
