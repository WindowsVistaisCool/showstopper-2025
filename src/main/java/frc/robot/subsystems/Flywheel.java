// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Flywheel extends SubsystemBase {

    private ThunderBird flywheelLeft;
    private ThunderBird flywheelRight;

    private final VelocityVoltage flywheelPID = new VelocityVoltage(0);

    private double targetRPS = 0;

    public Flywheel() {
        flywheelLeft = new ThunderBird(CAN.SHOOTER_LEFT_ID, CAN.CANBUS_FD, FlywheelConstants.LEFT_INVERT,
                FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT, false);
        flywheelRight = new ThunderBird(CAN.SHOOTER_RIGHT_ID, CAN.CANBUS_FD, FlywheelConstants.RIGHT_INVERT,
                FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT, false);

        flywheelLeft.configPIDF(0, FlywheelConstants.MOTOR_KP, FlywheelConstants.MOTOR_KI, FlywheelConstants.MOTOR_KD,
                FlywheelConstants.MOTOR_KA);
        flywheelRight.configPIDF(0, FlywheelConstants.MOTOR_KP, FlywheelConstants.MOTOR_KI, FlywheelConstants.MOTOR_KD,
                FlywheelConstants.MOTOR_KA);

        flywheelLeft.applyConfig();
        flywheelRight.applyConfig();

        initLogging();
    }

    public void setRPM(double RPM) {
        targetRPS = RPM / 60d;
    }

    public void stop() {
        setRPM(0d);
    }

    public void applyPower() {
        flywheelLeft.setControl(flywheelPID.withVelocity(targetRPS).withEnableFOC(false).withSlot(0));
        flywheelRight.setControl(flywheelPID.withVelocity(targetRPS).withEnableFOC(false).withSlot(0));
    }

    public void setRawPower(double power) {
        flywheelLeft.set(power);
        flywheelRight.set(power);
    }

    private void initLogging() {
        if (!DriverStation.isFMSAttached()) {
            LightningShuffleboard.setDoubleSupplier("Flywheel", "Target RPM", this::getTargetRPM);
            LightningShuffleboard.setDoubleSupplier("Flywheel", "Left Real RPM", this::getLeftRPM);
            LightningShuffleboard.setDoubleSupplier("Flywheel", "Right Real RPM", this::getRightRPM);
        }
    }

    public double getTargetRPM() {
        return targetRPS * 60d;
    }

    public double getLeftRPM() {
        return (flywheelLeft.getVelocity().getValueAsDouble() * 60d);
    }

    public double getRightRPM() {
        return (flywheelRight.getVelocity().getValueAsDouble() * 60d);
    }

    public double getAverageRPMs() {
        return (getLeftRPM() + getRightRPM()) / 2d;
    }

    @Override
    public void periodic() {
        applyPower();
    }
}
