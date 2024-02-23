package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class MoveClimberToBottom extends Command {
  private final ClimberSubsystem climber;

  public MoveClimberToBottom(ClimberSubsystem climber) {
    this.climber = climber;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {
    climber.setVoltage(ClimberConstants.CLIMBER_RESET_VOLTS.get());
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return climber.atBottom();
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }
}
