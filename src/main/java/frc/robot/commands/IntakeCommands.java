package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommands {

  private IntakeCommands() {}

  public static Command triggerIntake(
      IntakeSubsystem intakeSubsystem, DoubleSupplier speedSupplier) {
    return Commands.run(
        () ->
            intakeSubsystem.setVelocity(
                IntakeConstants.MAX_RAD_PER_SEC * speedSupplier.getAsDouble()),
        intakeSubsystem);
  }

  public static Command buttonIntake(
      IntakeSubsystem intakeSubsystem, BooleanSupplier isIntakeNote) {
    return Commands.run(
        () -> {
          if (isIntakeNote.getAsBoolean()) {
            intakeSubsystem.setVelocity(IntakeConstants.AUTO_RAD_PER_SEC);
          } else {
            intakeSubsystem.setVelocity(-IntakeConstants.AUTO_RAD_PER_SEC);
          }
        });
  }
}
