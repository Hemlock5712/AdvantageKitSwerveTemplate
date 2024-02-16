package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ResetClimbers extends SequentialCommandGroup {
  public ResetClimbers(ClimberSubsystem climber) {
    addCommands(
        new InstantCommand(climber::resetEncoder, climber),
        Commands.waitUntil(
                () -> Math.abs(climber.getPositionMeters()) > ClimberConstants.RESET_ROTATIONS)
            .raceWith(new MoveClimberToBottom(climber)),
        new ConditionalCommand(
            new InstantCommand(),
            new InstantCommand(climber::toggleInvert, climber)
                .andThen(new MoveClimberToBottom(climber)),
            climber::atBottom),
        new InstantCommand(climber::resetEncoder, climber));
  }
}
