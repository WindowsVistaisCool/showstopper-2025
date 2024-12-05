// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.thunder.LightningRobot;

public class Robot extends LightningRobot {

    public Robot() {
        super(new RobotContainer());
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
        
        RobotContainer robotContainer = (RobotContainer) getContainer();

        if (!RobotContainer.DRIVETRAIN_DISABLED) {
            // realistically this doesn't work because we aren't running 10ms cycles
            robotContainer.getDrivetrain().brake();
        }
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
    }
}
