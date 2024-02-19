package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ResetClimberBasic extends SequentialCommandGroup {
  public ResetClimberBasic(ClimberSubsystem climber) {
    addCommands(
        new MoveClimberToBottom(climber),
        new InstantCommand(climber::toggleInvert, climber),
        new InstantCommand(climber::resetEncoder, climber));
  }
}
