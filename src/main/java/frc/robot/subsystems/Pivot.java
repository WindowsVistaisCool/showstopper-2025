package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.PivotPoses;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Pivot extends SubsystemBase {

    private ThunderBird leftMotor;
    private ThunderBird rightMotor;
    private CANcoder angleEncoder;

    private double targetAngle = PivotPoses.HOME;

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

        TalonFXConfiguration motorConfig = leftMotor.getConfig();
        motorConfig.Slot0.kP = PivotConstants.MOTOR_KP;
        motorConfig.Slot0.kI = PivotConstants.MOTOR_KI;
        motorConfig.Slot0.kD = PivotConstants.MOTOR_KD;
        motorConfig.Slot0.kS = PivotConstants.MOTOR_KS;
        motorConfig.Slot0.kV = PivotConstants.MOTOR_KV;
        motorConfig.Slot0.kA = PivotConstants.MOTOR_KA;
        motorConfig.Slot0.kG = PivotConstants.MOTOR_KG;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

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
        targetAngle = MathUtil.clamp(angle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE);

        applyAngle();
    }

    private void applyAngle() {        
        leftMotor.setControl(positionPID.withPosition(targetAngle).withSlot(0));
    }

    public boolean onTarget() {
        return Math.abs(getAngle() - getTargetAngle()) < PivotConstants.ANGLE_TOLERANCE;
    }

    public double getAngle() {
        return angleEncoder.getPosition().getValueAsDouble();
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * does NOT set raw power 
     */
    public void setRawPower(double power) {
        power *= 0.4;
        leftMotor.set(power);
    }

    @Override
    public void periodic() {
        targetAngle = LightningShuffleboard.getDouble("Pivot", "INPUT target angle", targetAngle);

        // applyAngle();
        //
        // setRawPower(0);
    }
}
