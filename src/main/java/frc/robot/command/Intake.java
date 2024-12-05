// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;

public class Intake extends Command {

    private Indexer indexer;
    
    private boolean reverse;

    public Intake(Indexer indexer, boolean reverse) {
        this.indexer = indexer;
        this.reverse = reverse;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setPower(IndexerConstants.DEFAULT_INDEXER_POWER * (reverse ? -1d : 1d));
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
