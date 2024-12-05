// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PivotRequest extends Command {

    private Pivot pivot;

    private DoubleSupplier angleRequest;

    public PivotRequest(Pivot pivot, DoubleSupplier angleRequest) {
        this.pivot = pivot;
        this.angleRequest = angleRequest;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        pivot.setAngle(angleRequest.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
