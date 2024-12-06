package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Pivot extends SubsystemBase {

    private ThunderBird leftMotor;
    private ThunderBird rightMotor;
    private CANcoder angleEncoder;

    private double targetAngle = 0;

    private final PositionVoltage positionPID = new PositionVoltage(0d);

    public Pivot() {

        CANcoderConfiguration angleConfig = new CANcoderConfiguration();
        angleConfig.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(PivotConstants.ENCODER_OFFSET)
                .withSensorDirection(PivotConstants.ENCODER_DIRECTION);

        angleEncoder = new CANcoder(CAN.PIVOT_CANCODER_ID, CAN.CANBUS_FD);
        angleEncoder.getConfigurator().apply(angleConfig);

        leftMotor = new ThunderBird(CAN.PIVOT_LEFT_ID, CAN.CANBUS_FD, PivotConstants.MOTOR_LEFT_INVERT,
                PivotConstants.MOTOR_STATOR_CURRENT_LIMIT, PivotConstants.MOTOR_BRAKE_MODE);
        rightMotor = new ThunderBird(CAN.PIVOT_RIGHT_ID, CAN.CANBUS_FD, PivotConstants.MOTOR_RIGHT_INVERT,
                PivotConstants.MOTOR_STATOR_CURRENT_LIMIT, PivotConstants.MOTOR_BRAKE_MODE);

        leftMotor.configPIDF(0, PivotConstants.MOTOR_KP, PivotConstants.MOTOR_KI,
                PivotConstants.MOTOR_KD, PivotConstants.MOTOR_KS, PivotConstants.MOTOR_KV);

        rightMotor.configPIDF(0, PivotConstants.MOTOR_KP, PivotConstants.MOTOR_KI,
                PivotConstants.MOTOR_KD, PivotConstants.MOTOR_KS, PivotConstants.MOTOR_KV);
        TalonFXConfiguration motorConfig = leftMotor.getConfig();

        motorConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = PivotConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = PivotConstants.ROTOR_TO_ENCODER_RATIO;

        MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.MAGIC_CRUISE_VEL;
        motionMagicConfigs.MotionMagicAcceleration = PivotConstants.MAGIC_ACCEL;
        motionMagicConfigs.MotionMagicJerk = PivotConstants.MAGIC_JERK;

        leftMotor.applyConfig(motorConfig);

        rightMotor.setControl(new Follower(CAN.PIVOT_LEFT_ID, true));

        initLogging();
    }

    private void initLogging() {
        LightningShuffleboard.setDoubleSupplier("Pivot", "Current Angle", this::getAngle);
        LightningShuffleboard.setDoubleSupplier("Pivot", "Target Angle", this::getTargetAngle);
    }

    public void setAngle(double angle) {
        targetAngle = MathUtil.clamp(angle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE) / 360d;
    }

    private void applyAngle() {
        leftMotor.setControl(positionPID.withPosition(targetAngle).withSlot(0));
    }

    public double getAngle() {
        // this is also wrong!!
        return angleEncoder.getPosition().getValueAsDouble() / 409.6;//PivotConstants.ROTOR_TO_ENCODER_RATIO;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setRawPower(double power) {
        power *= 0.4;
        leftMotor.set(power);
    }

    @Override
    public void periodic() {
        targetAngle = LightningShuffleboard.getDouble("Pivot", "INPUT target angle", targetAngle);

        // applyAngle();
    }
}
