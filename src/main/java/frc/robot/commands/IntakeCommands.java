package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommands {

  private IntakeCommands() {}

  public static Command triggerIntake(Intake intakeSubsystem, DoubleSupplier speedSupplier) {
    return Commands.run(
        () ->
            intakeSubsystem.runVelocity(
                IntakeConstants.MAX_RAD_PER_SEC * speedSupplier.getAsDouble()),
        intakeSubsystem);
  }

  public static Command buttonIntake(Intake intakeSubsystem, BooleanSupplier isIntakeNote) {
    return Commands.run(
        () -> {
          if (isIntakeNote.getAsBoolean()) {
            intakeSubsystem.runVelocity(IntakeConstants.AUTO_RAD_PER_SEC);
          } else {
            intakeSubsystem.runVelocity(-IntakeConstants.AUTO_RAD_PER_SEC);
          }
        });
  }
}
