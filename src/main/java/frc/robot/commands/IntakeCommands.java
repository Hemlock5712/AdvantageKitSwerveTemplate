package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommands {
  private IntakeCommands() {}

  public static Command untilNote(Intake intake, BooleanSupplier detectingNoteSupplier) {
    return Commands.startEnd(
            () -> {
              intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE.get());
            },
            intake::stop,
            intake)
        .until(detectingNoteSupplier);
  }

  public static Command manualIntakeCommand(
      Intake intake, DoubleSupplier speed) {
    return Commands.runEnd(
        () -> {
          double volts =
              IntakeConstants.INTAKE_VOLTAGE.get() * MathUtil.clamp(speed.getAsDouble(), -1, 1);


          intake.setVoltage(volts);
        },
        intake::stop,
        intake);
  }
}
