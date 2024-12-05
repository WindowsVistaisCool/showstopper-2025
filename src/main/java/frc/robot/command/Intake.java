// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;

public class Intake extends Command {

    private Indexer indexer;
    private Flywheel flywheel;

    private boolean reverse;

    public Intake(Indexer indexer, Flywheel flywheel, boolean reverse) {
        this.indexer = indexer;
        this.flywheel = flywheel; // not moving the subsystem; don't need to require
        this.reverse = reverse;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setPower(IndexerConstants.DEFAULT_INDEXER_POWER * (reverse ? -1d : 1d)); // multiply by -1 if reverse
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop(); // stop indexer on command end
    }

    @Override
    public boolean isFinished() {
        // stop if the flywheel is not running and our indexer is stalling
        return flywheel.getAverageRPMs() <= 1000d && indexer.getStalling();
    }
}
