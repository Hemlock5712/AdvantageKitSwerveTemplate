package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ResetClimberBasic {
  private ResetClimberBasic() {}
  ;

  public static Command on(ClimberSubsystem climber) {
    return new ConditionalCommand(
        Commands.none(),
        new MoveClimberToBottom(climber)
            .andThen(
                new InstantCommand(climber::resetClimbersAssumingPositiveVoltageIsDown, climber)),
        climber::isBeenReset);
  }
}
