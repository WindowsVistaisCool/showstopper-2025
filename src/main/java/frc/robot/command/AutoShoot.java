// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;

public class AutoShoot extends SequentialCommandGroup {
  public AutoShoot(Pivot pivot, Flywheel flywheel, Indexer indexer, double angleRequest, double targetRPM) {
    addCommands(
            new ParallelDeadlineGroup(new PivotRequest(pivot, () -> angleRequest).until(pivot::onTarget), new FlywheelRequest(flywheel, () -> targetRPM)),
            new Intake(indexer, flywheel, true)
        );
  }
}
