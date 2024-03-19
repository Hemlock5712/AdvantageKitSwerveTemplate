package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class AdjustPositionCommands {
  private AdjustPositionCommands() {}

  public static Command setPose(Drive drive, Supplier<Pose2d> poseSupplier) {
    return Commands.runOnce(() -> drive.setAutoStartPose(poseSupplier.get())).ignoringDisable(true);
  }

  public static Command setTranslation(Drive drive, Supplier<Translation2d> translationSupplier) {
    return Commands.runOnce(
            () ->
                drive.setAutoStartPose(
                    new Pose2d(translationSupplier.get(), drive.getPose().getRotation())))
        .ignoringDisable(true);
  }

  public static Command setRotation(Drive drive, Supplier<Rotation2d> rotationSupplier) {
    return Commands.runOnce(
            () ->
                drive.setAutoStartPose(
                    new Pose2d(drive.getPose().getTranslation(), rotationSupplier.get())))
        .ignoringDisable(true);
  }
}
