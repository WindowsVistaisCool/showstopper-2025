// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class SetFlywheelsRPM extends Command {

    private Flywheel flywheel;
    
    private DoubleSupplier targetRPMSupplier;

    public SetFlywheelsRPM(Flywheel flywheel, DoubleSupplier targetRPM) {
        this.flywheel = flywheel;
        this.targetRPMSupplier = targetRPM;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        flywheel.setRPM(targetRPMSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setRPM(0d);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
