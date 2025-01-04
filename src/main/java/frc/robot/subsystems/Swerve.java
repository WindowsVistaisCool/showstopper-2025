package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DrivetrainConstants;
import frc.thunder.util.Pose4d;
import frc.robot.Constants.AutonomousConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {

    /* SwerveRequests */
    private SwerveRequest.FieldCentric driveField;
    private SwerveRequest.RobotCentric driveRobot;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    
    /* Drivetrain Constants */
    private double maxSpeed = DrivetrainConstants.MaxSpeed;
    private double maxAngularRate = DrivetrainConstants.MaxAngularRate * DrivetrainConstants.ROT_MULT;
    private double speedMult = DrivetrainConstants.NORMAL_SPEED_MULT;
    private double angularMult = DrivetrainConstants.NORMAL_ROT_MULT;

    /* Slowmode */
    private boolean slowMode = false;

    /* Sim */
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        driveField = new SwerveRequest.FieldCentric();
        driveRobot = new SwerveRequest.RobotCentric();

        configurePathPlanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        driveField = new SwerveRequest.FieldCentric();
        driveRobot = new SwerveRequest.RobotCentric();

        configurePathPlanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public void applyVisionPose(Pose4d pose) {
        addVisionMeasurement(pose.toPose2d(), pose.getFPGATimestamp(), pose.getStdDevs());
    }

    private void configurePathPlanner() {
        
    AutoBuilder.configureHolonomic(() -> getPose(), // Supplier of current robot pose
            this::seedFieldRelative, // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds, (speeds) -> this.setControl(autoRequest.withSpeeds(speeds).withDriveRequestType(DriveRequestType.Velocity)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(AutonomousConstants.TRANSLATION_PID, AutonomousConstants.ROTATION_PID, AutonomousConstants.MAX_MODULE_VELOCITY, AutonomousConstants.DRIVE_BASE_RADIUS, AutonomousConstants.REPLANNING_CONFIG, AutonomousConstants.CONTROL_LOOP_PERIOD), () -> {
                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, this); // Subsystem for requirements
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Pose2d getPose() {
        var state = getState();
        if (state == null || getState().Pose == null) {
            return new Pose2d();
        }
        return state.Pose;
    }

    public double getSpeedMult() {
        return speedMult;
    }

    public double getRotMult() {
        return angularMult;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getMaxAngularRate() {
        return maxAngularRate;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /* Slowmode methods */
    private void setSlowMode(boolean state) {
        slowMode = state;
        if (state) {
            speedMult = DrivetrainConstants.SLOW_SPEED_MULT;
            angularMult = DrivetrainConstants.SLOW_ROT_MULT;
        } else {
            speedMult = DrivetrainConstants.NORMAL_SPEED_MULT;
            angularMult = DrivetrainConstants.NORMAL_ROT_MULT;
        }
    }

    public Command enableSlowMode() {
        return runOnce(() -> setSlowMode(true));
    }

    public Command disableSlowMode() {
        return runOnce(() -> setSlowMode(false));
    }

    public boolean getSlowMode() {
        return slowMode;
    }

    /* DRIVE METHODS */

    /**
     * Apply a generic swerve request to the drivetrain
     * @param requestSupplier Swerve Request
     * @return request to drive
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Apply a percentage Field centric request to the drivetrain
     *
     * @param x   the x, percent of max velocity (-1,1)
     * @param y   the y, percent of max velocity (-1,1)
     * @param rot the rotational, percent of max velocity (-1,1)
     * @return the request to drive for the drivetrain
     */
    public Command applyPercentRequestField(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return run(() -> this.setControl(driveField.withVelocityX(x.getAsDouble() * maxSpeed)
                .withVelocityY(y.getAsDouble() * maxSpeed).withRotationalRate(rot.getAsDouble() * maxAngularRate)
                .withDriveRequestType(DriveRequestType.Velocity)));
    }

    /**
     * Apply a Field centric request to the drivetrain run in periodic
     *
     * @param x   the x velocity m/s
     * @param y   the y velocity m/s
     * @param rot the rotational velocity in rad/s
     */
    public void setField(double x, double y, double rot) {
        this.setControl(driveField.withVelocityX(x).withVelocityY(y).withRotationalRate(rot)
                .withDriveRequestType(DriveRequestType.Velocity));
    }

    /**
     * Apply a Field centric request to the drivetrain run in periodic, Allows
     * driving normally and pid control of rotation
     *
     * @param x   the x, percent of max velocity (-1,1)
     * @param y   the y, percent of max velocity (-1,1)
     * @param rot the rotational, percent of max velocity rad/s
     */
    public void setFieldDriver(double x, double y, double rot) {
        this.setControl(driveField.withVelocityX(x * maxSpeed).withVelocityY(y * maxSpeed).withRotationalRate(rot)
                .withDriveRequestType(DriveRequestType.Velocity));
    }

    /**
     * Apply a percentage Robot centric request to the drivetrain
     *
     * @param x   the x, percent of max velocity (-1,1)
     * @param y   the y, percent of max velocity (-1,1)
     * @param rot the rotational, percent of max velocity (-1,1)
     * @return the request to drive for the drivetrain
     */
    public Command applyPercentRequestRobot(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return run(() -> this.setControl(driveRobot.withVelocityX(x.getAsDouble() * maxSpeed)
                .withVelocityY(y.getAsDouble() * maxSpeed).withRotationalRate(rot.getAsDouble() * maxAngularRate)
                .withDriveRequestType(DriveRequestType.Velocity)));
    }

    /**
     * Apply a Robot centric request to the drivetrain run in periodic
     *
     * @param x   the x velocity m/s
     * @param y   the y velocity m/s
     * @param rot the rotational velocity in rad/s
     */
    public void setRobot(double x, double y, double rot) {
        this.setControl(driveRobot.withVelocityX(x).withVelocityY(y).withRotationalRate(rot)
                .withDriveRequestType(DriveRequestType.Velocity));
    }

    /**
     * Sets the robot in park mode
     */
    public void brake() {
        this.setControl(brake);
    }

    public Command setBrake() {
        return run(() -> this.setControl(brake));
    }

    /**
     * Resets the field relative heading to the current robot heading
     * 
     * @return the SequentialCommandGroup to reset the field relative heading
     */
    public Command resetForward() {
        return runOnce(this::seedFieldRelative).andThen(
                runOnce(() -> this.setOperatorPerspectiveForward(new Rotation2d(Math.toRadians(0)))));
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /*
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state
         */
        /*
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match
         */
        /*
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled
         */
        /*
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
}
