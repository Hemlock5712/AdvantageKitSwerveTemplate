package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class NotePickup {
  private NotePickup() {}

  /**
   * @param drive
   * @param intake
   * @param hasNote
   * @param targetSupplier target is in robot odometry space (not vision)
   * @return
   */
  public static Command pickupNote(
      Drive drive, Intake intake, BooleanSupplier hasNote, Supplier<Translation2d> targetSupplier) {
    final ProfiledPIDController thetaController = drive.getThetaController();

    return Commands.runOnce(() -> intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE.get()))
        .andThen(
            Commands.runEnd(
                () -> {
                  thetaController.setGoal(
                      new Pose2d(targetSupplier.get(), new Rotation2d())
                          .relativeTo(drive.getDrive())
                          .getRotation()
                          .getRadians());
                },
                () -> {
                  intake.stop();
                  drive.stop();
                },
                drive,
                intake))
        .until(hasNote);
  }
}
