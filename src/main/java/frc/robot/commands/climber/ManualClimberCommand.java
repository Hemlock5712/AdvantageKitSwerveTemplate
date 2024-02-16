package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;

import java.util.function.DoubleSupplier;


public class ManualClimberCommand extends Command {
    private final ClimberSubsystem climber;
    private final DoubleSupplier speedPercentSupplier;

    public ManualClimberCommand(ClimberSubsystem climber, DoubleSupplier speedPercentSupplier) {
        this.climber = climber;
        this.speedPercentSupplier = speedPercentSupplier;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double direction = climber.pastFullExtension() ? -1 : 1;
        // double direction = 1;

        climber.setVoltage(
            direction *
            ClimberConstants.CLIMBER_VOLTS *
            speedPercentSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
