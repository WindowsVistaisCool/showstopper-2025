package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class FlywheelRequest extends Command {

    private Flywheel flywheel;
    
    private DoubleSupplier targetRPMSupplier;

    public FlywheelRequest(Flywheel flywheel, DoubleSupplier targetRPM) {
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
