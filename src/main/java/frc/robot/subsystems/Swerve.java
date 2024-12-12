package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

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

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {

    /* SwerveRequests */
    private SwerveRequest.FieldCentric driveField;
    private SwerveRequest.RobotCentric driveRobot;
    // private SwerveRequest.ApplyChassisSpeeds autoRequest = new
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

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public void applyVisionPose(Pose4d pose) {
        addVisionMeasurement(pose.toPose2d(), pose.getFPGATimestamp(), pose.getStdDevs());
    }

    private void configurePathPlanner() {
        // TODO: implement :)
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
     * 
     * @param requestSupplier Swerve Request
     * @return request to drive
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Apply a percentage Field centric request to the drivetrain
     *
     * @param request the swerve request used to apply speeds
     * @param x       the x, percent of max velocity (-1,1)
     * @param y       the y, percent of max velocity (-1,1)
     * @param rot     the rotational, percent of max velocity (-1,1)
     * @return the request to drive for the drivetrain
     */
    public Command applyPercentRequestField(FieldCentric request, DoubleSupplier x, DoubleSupplier y,
            DoubleSupplier rot) {
        return run(() -> this.setField(request, x.getAsDouble(), y.getAsDouble(), rot.getAsDouble()));
    }

    public Command applyPercentRequestField(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return applyPercentRequestField(driveField, x, y, rot);
    }

    /**
     * Applies a fieldcentric request with angular and speed mults
     * 
     * @param request   fieldcentric request to apply
     * @param x         the x, percent of max velocity (-1,1)
     * @param y         the y, percent of max velocity (-1,1)
     * @param rot       the rotational, percent of max velocity (-1,1)
     * @return the request to drive
     */
    public Command applyFieldWithLimits(FieldCentric request, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return applyPercentRequestField(request, () -> x.getAsDouble() * this.getSpeedMult(),
                () -> y.getAsDouble() * this.getSpeedMult(), () -> rot.getAsDouble() * this.getRotMult());
    }

    /**
     * Apply a Field centric request to the drivetrain run in periodic
     *
     * @param request the fieldcentric request to apply
     * @param x       the x velocity m/s
     * @param y       the y velocity m/s
     * @param rot     the rotational velocity in rad/s
     */
    public void setField(FieldCentric request, double x, double y, double rot) {
        this.setControl(request.withVelocityX(x * maxSpeed).withVelocityY(y * maxSpeed)
                .withRotationalRate(rot * maxAngularRate)
                .withDriveRequestType(DriveRequestType.Velocity));
    }

    /**
     * Apply a percentage Robot centric request to the drivetrain
     *
     * @param request the robot centric swerverequest
     * @param x       the x, percent of max velocity (-1,1)
     * @param y       the y, percent of max velocity (-1,1)
     * @param rot     the rotational, percent of max velocity (-1,1)
     * @return the request to drive for the drivetrain
     */
    public Command applyPercentRequestRobot(RobotCentric request, DoubleSupplier x, DoubleSupplier y,
            DoubleSupplier rot) {
        return run(() -> this.setRobot(request, x.getAsDouble(), y.getAsDouble(), rot.getAsDouble()));
    }

    public Command applyPercentRequestRobot(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return applyPercentRequestRobot(driveRobot, x, y, rot);
    }

    /**
     * Applies a robotcentric request with angular and speed mults
     * 
     * @param request   robotcentric request to apply
     * @param x         the x, percent of max velocity (-1,1)
     * @param y         the y, percent of max velocity (-1,1)
     * @param rot       the rotational, percent of max velocity (-1,1)
     * @return the request to drive
     */
    public Command applyRobotWithLimits(RobotCentric request, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return applyPercentRequestRobot(request, () -> x.getAsDouble() * this.getSpeedMult(),
                () -> y.getAsDouble() * this.getSpeedMult(), () -> rot.getAsDouble() * this.getRotMult());
    }

    /**
     * Apply a Robot centric request to the drivetrain run in periodic
     *
     * @param request the robot centric swerverequest
     * @param x       the x velocity m/s
     * @param y       the y velocity m/s
     * @param rot     the rotational velocity in rad/s
     */
    public void setRobot(RobotCentric request, double x, double y, double rot) {
        this.setControl(
                request.withVelocityX(x * maxSpeed).withVelocityY(y * maxSpeed).withRotationalRate(rot * maxAngularRate)
                        .withDriveRequestType(DriveRequestType.Velocity));
    }

    /**
     * Sets the robot in park mode
     */
    public void brake() {
        this.setControl(brake);
    }

    /**
     * Robot park as a command
     */
    public Command setBrake() {
        return run(this::brake);
    }

    /**
     * Resets the field relative heading to the current robot heading
     * 
     * @return the SequentialCommandGroup to reset the field relative heading
     */
    public Command resetForward() {
        // return runOnce(this::seedFieldCentric).andThen(
        // runOnce(() -> this.setOperatorPerspectiveForward(new
        // Rotation2d(Math.toRadians(0)))));
        return null;
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
