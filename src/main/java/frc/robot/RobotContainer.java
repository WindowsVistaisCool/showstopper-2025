// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LL;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.thunder.LightningContainer;
import frc.thunder.filter.XboxControllerFilter;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.PivotConstants.PivotPoses;
import frc.robot.command.AutoShoot;
import frc.robot.command.Intake;
import frc.robot.command.PivotRequest;
import frc.robot.command.FlywheelRequest;
import frc.thunder.shuffleboard.LightningShuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class RobotContainer extends LightningContainer {

    private XboxControllerFilter driver;
    private XboxControllerFilter copilot;

    public static final boolean DRIVETRAIN_DISABLED = false;
    //0 for LL, 1 for PV, 2 for disabled
    public static final int VISION_DISABLED = 0;

    private SendableChooser<Command> autoChooser;

    private LL limelight;
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


        this.pivot = new Pivot();
        this.indexer = new Indexer();
        this.flywheel = new Flywheel();


        this.vision = new PhotonVision();
        this.limelight = new LL();


        // limelight.setApplyVisionUpdate(drivetrain::applyVisionPose);
    }

    @Override
    protected void configureButtonBindings() {
        // if (!DRIVETRAIN_DISABLED) {
        //     new Trigger(() -> driver.getLeftTriggerAxis() > 0.25d).whileTrue(drivetrain.applyRequest(
        //             driveRobotCentric,
        //             () -> -driver.getLeftX(),
        //             () -> -driver.getLeftY(),
        //             () -> -driver.getRightX()));
        //
        //     new Trigger(() -> driver.getRightTriggerAxis() > 0.25d)
        //             .onTrue(drivetrain.enableSlowMode())
        //             .onFalse(drivetrain.disableSlowMode());
        //
        //     // new Trigger(() -> driver.getStartButton() &&
        //     // driver.getBackButton()).onTrue(drivetrain.resetForward());
        //
        //     new Trigger(driver::getXButton).onTrue(drivetrain.setBrake());
        // }


        if (!DRIVETRAIN_DISABLED) {
            new Trigger(() -> driver.getLeftTriggerAxis() > 0.25d).whileTrue(
                    drivetrain.applyRequest(
                            () -> driveRobotCentric.withRotationalRate(-driver.getRightX() * drivetrain.getMaxAngularRate())
                                    .withVelocityX(-driver.getLeftY() * drivetrain.getMaxSpeed())
                                    .withVelocityY(-driver.getLeftX() * drivetrain.getMaxSpeed())));
    
            new Trigger(() -> driver.getRightTriggerAxis() > 0.25d)
                    .onTrue(drivetrain.enableSlowMode())
                    .onFalse(drivetrain.disableSlowMode());
    
            // new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(drivetrain.resetForward());
    
            new Trigger(driver::getXButton).onTrue(drivetrain.setBrake());
        }

        // Intake Controls
        new Trigger(copilot::getLeftBumper).whileTrue(new Intake(indexer, flywheel, true));
        new Trigger(copilot::getRightBumper).whileTrue(new Intake(indexer, flywheel, false));

        new Trigger(copilot::getBButton).whileTrue(new PivotRequest(pivot, () -> PivotPoses.SHOOT_3));
        new Trigger(copilot::getYButton).whileTrue(new PivotRequest(pivot, () -> PivotPoses.SHOOT_2));
        new Trigger(copilot::getXButton).whileTrue(new PivotRequest(pivot, () -> PivotPoses.SHOOT_1));

        new Trigger(copilot::getAButton).whileTrue(new FlywheelRequest(flywheel, () -> FlywheelConstants.SHOOT_SPEED));
        // new Trigger(copilot::getAButton).whileTrue(new RunCommand(() ->
        // flywheel.setRawPower(1d), flywheel));
        //
    }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
        drivetrain.applyPercentRequestField(() -> -(driver.getLeftY() *
        drivetrain.getSpeedMult()),
        () -> -(driver.getLeftX() * drivetrain.getSpeedMult()),
        () -> -(driver.getRightX() * drivetrain.getRotMult())));

        // if (!DRIVETRAIN_DISABLED) {
        //     drivetrain.setDefaultCommand(drivetrain.applyRequest(
        //             driveFieldCentric,
        //             () -> -driver.getLeftX(),
        //             () -> -driver.getLeftY(),
        //             () -> -driver.getRightX()));
        //
        //     if (!VISION_DISABLED) {
        //         vision.setDefaultCommand(vision.updateOdometry(drivetrain));
        //     }
        //
        //     drivetrain.registerTelemetry(logger::telemeterize);
        // }
        //

            drivetrain.registerTelemetry(logger::telemeterize);
        

        // flywheel.setDefaultCommand(new RunCommand(() ->
        // flywheel.setPower(copilot.getRightTriggerAxis() -
        // copilot.getLeftTriggerAxis()), flywheel));

        // raw dutycycle movement
        // pivot.setDefaultCommand(new RunCommand(() -> pivot.setRawPower(copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()), pivot));

        // angle set based on right trigger (please fix pivot angle math before using!)
        // double ANGLE_MULT = 10; // range: 0-10 
        pivot.setDefaultCommand(new PivotRequest(pivot, () -> (PivotPoses.HOME)));

    }

    @Override
    protected void initializeNamedCommands() {

        NamedCommands.registerCommand("Shoot", new AutoShoot(pivot, flywheel, indexer, PivotPoses.SHOOT_2, FlywheelConstants.SHOOT_SPEED));
        NamedCommands.registerCommand("ReadyShoot", new ParallelDeadlineGroup(new PivotRequest(pivot, () -> PivotPoses.SHOOT_2).until(pivot::onTarget), new FlywheelRequest(flywheel, () -> FlywheelConstants.SHOOT_SPEED)));
        NamedCommands.registerCommand("IndexUp", new Intake(indexer, flywheel, true));


	autoChooser = AutoBuilder.buildAutoChooser();
	LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);
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
        return autoChooser.getSelected();
    }

    public Swerve getDrivetrain() {
        return drivetrain == null ? TunerConstants.DriveTrain : drivetrain;
    }
}
