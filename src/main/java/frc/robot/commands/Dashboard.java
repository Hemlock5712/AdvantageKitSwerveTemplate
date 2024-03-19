package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Arrays;
import java.util.function.Supplier;

public class Dashboard {
  private Dashboard() {}

  public static Command logField(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d[]> noteSupplier) {
    final var field = new Field2d();
    SmartDashboard.putData(field);
    return Commands.run(
            () -> {
              field.setRobotPose(robotPoseSupplier.get());
              field
                  .getObject("notes")
                  .setPoses(
                      Arrays.stream(noteSupplier.get())
                          .map(translation2d -> new Pose2d(translation2d, new Rotation2d()))
                          .toList());
            })
        .ignoringDisable(true);
  }
}
