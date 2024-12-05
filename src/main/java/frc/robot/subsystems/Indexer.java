// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Indexer extends SubsystemBase {

    // private TalonFX indexerMotor;
    private ThunderBird indexerMotor;

    public Indexer() {
        // indexerMotor = new TalonFX(RobotMap.CAN.INDEXER_ID);
        indexerMotor = new ThunderBird(CAN.INDEXER_ID, CAN.CANBUS_FD, IndexerConstants.INDEXER_INVERT,
                IndexerConstants.MOTOR_STATOR_CURRENT_LIMIT, false);

        initLogging();
    }

    public void setPower(double power) {
        // indexerMotor.set(power);
        indexerMotor.setControl(new DutyCycleOut(power));
    }

    public void stop() {
        setPower(0d);
    }

    private void initLogging() {
        LightningShuffleboard.setDoubleSupplier("Indexer", "Power",
                () -> indexerMotor.getDutyCycle().getValueAsDouble());
    }

    @Override
    public void periodic() {
    }
}
