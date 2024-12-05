// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.thunder.LightningContainer;
import frc.thunder.filter.XboxControllerFilter;
import frc.robot.Constants.TunerConstants;
import frc.robot.command.Intake;
import frc.robot.command.PivotRequest;

public class RobotContainer extends LightningContainer {

    private XboxControllerFilter driver;
    private XboxControllerFilter copilot;

    public static final boolean DRIVETRAIN_DISABLED = true;
    public static final boolean VISION_DISABLED = true;

    private Swerve drivetrain;
    private PhotonVision vision;
    private Pivot pivot;
    private Indexer indexer;
    private Flywheel flywheel;
    private Telemetry logger;

    private SwerveRequest.FieldCentric driveFieldCentric;
    private SwerveRequest.RobotCentric driveRobotCentric;

    @Override
    protected void initializeSubsystems() {
        this.driver = new XboxControllerFilter(0, 0.1, -1, 1, XboxControllerFilter.filterMode.SQUARED);
        this.copilot = new XboxControllerFilter(1, 0.1, -1, 1, XboxControllerFilter.filterMode.SQUARED);

        if (!DRIVETRAIN_DISABLED) {
            this.drivetrain = getDrivetrain();
            this.logger = new Telemetry(drivetrain.getMaxSpeed());

            this.driveFieldCentric = new SwerveRequest.FieldCentric();
            this.driveRobotCentric = new SwerveRequest.RobotCentric();
        }
        if (!VISION_DISABLED) {
            this.vision = new PhotonVision();
        }
        this.pivot = new Pivot();
        this.indexer = new Indexer();
        this.flywheel = new Flywheel();
    }

    @Override
    protected void configureButtonBindings() {
        // old old code probably do not use
        // new Trigger(() -> driver.getLeftTriggerAxis() > 0.25d).whileTrue(
        // drivetrain.applyPercentRequestRobot(
        // () -> -driver.getLeftY(),
        // () -> -driver.getLeftX(),
        // () -> -driver.getRightX()));

        if (!DRIVETRAIN_DISABLED) {
            new Trigger(() -> driver.getLeftTriggerAxis() > 0.25d).whileTrue(
                    drivetrain.applyRequest(
                            () -> driveRobotCentric.withRotationalRate(-driver.getRightX() * drivetrain.getMaxAngularRate())
                                    .withVelocityX(-driver.getLeftY() * drivetrain.getMaxSpeed())
                                    .withVelocityY(-driver.getLeftX() * drivetrain.getMaxSpeed())));
    
            new Trigger(() -> driver.getRightTriggerAxis() > 0.25d)
                    .onTrue(drivetrain.enableSlowMode())
                    .onFalse(drivetrain.disableSlowMode());
    
            new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(drivetrain.resetForward());
    
            new Trigger(driver::getXButton).onTrue(drivetrain.setBrake());
        }

        // Intake Controls
        new Trigger(copilot::getLeftBumper).whileTrue(new Intake(indexer, flywheel, true));
        new Trigger(copilot::getRightBumper).whileTrue(new Intake(indexer, flywheel, false));

        // new Trigger(copilot::getAButton).whileTrue(new SetFlywheelsRPM(flywheel, () -> 6000d));
        new Trigger(copilot::getAButton).whileTrue(new RunCommand(() -> flywheel.setRawPower(1d), flywheel));
    }

    @Override
    protected void configureDefaultCommands() {
        // drivetrain.setDefaultCommand(
        //         drivetrain.applyPercentRequestField(() -> -(driver.getLeftY() * drivetrain.getSpeedMult()),
        //                 () -> -(driver.getLeftX() * drivetrain.getSpeedMult()),
        //                 () -> -(driver.getRightX() * drivetrain.getRotMult())));
    
        if (!DRIVETRAIN_DISABLED) {
            drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                    () -> driveFieldCentric.withRotationalRate(-driver.getRightX() * drivetrain.getMaxAngularRate())
                        .withVelocityX(-driver.getLeftY() * drivetrain.getMaxSpeed())
                        .withVelocityY(-driver.getLeftX() * drivetrain.getMaxSpeed())));
    
            if (!VISION_DISABLED) {
                vision.setDefaultCommand(vision.updateOdometry(drivetrain));
            }

            drivetrain.registerTelemetry(logger::telemeterize);
        }

        // flywheel.setDefaultCommand(new RunCommand(() -> flywheel.setPower(copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()), flywheel));

        // raw dutycycle movement
        pivot.setDefaultCommand(new RunCommand(() -> pivot.setRawPower(copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()), pivot));

        // angle set based on right trigger (please fix pivot angle math before using!)
        double ANGLE_MULT = 10; // range: 0-10 
        // pivot.setDefaultCommand(new PivotRequest(pivot, () -> (copilot.getRightTriggerAxis() * ANGLE_MULT)));

    }

    @Override
    protected void initializeNamedCommands() {
    }

    @Override
    protected void configureSystemTests() {
    }

    @Override
    protected void releaseDefaultCommands() {
    }

    @Override
    protected void initializeDashboardCommands() {
    }

    @Override
    protected void configureFaultCodes() {
    }

    @Override
    protected void configureFaultMonitors() {
    }

    @Override
    protected Command getAutonomousCommand() {
        return null;
    }

    public Swerve getDrivetrain() {
        return drivetrain == null ? TunerConstants.DriveTrain : drivetrain;
    }
}
