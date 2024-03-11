package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class DriveToAmp extends ConditionalCommand {
  public DriveToAmp(Supplier<Pose2d> robotPoseSupplier) {
    super(
        createCloseAlignCommand(robotPoseSupplier),
        new DriveToPoint(FieldConstants.ampScoringPose),
        () ->
            1
                > robotPoseSupplier
                    .get()
                    .getTranslation()
                    .getDistance(
                        AllianceFlipUtil.apply(FieldConstants.ampScoringPose.getTranslation())));
  }

  private static Command createCloseAlignCommand(Supplier<Pose2d> robotPoseSupplier) {
    return Commands.defer(
        () -> {
          var path =
              PathPlannerPath.fromPathPoints(
                  List.of(
                      new PathPoint(
                          robotPoseSupplier.get().getTranslation().minus(new Translation2d(0, 1))),
                      new PathPoint(
                          AllianceFlipUtil.apply(FieldConstants.ampScoringPose).getTranslation())),
                  DriveConstants.pathPlannerConstraints,
                  new GoalEndState(0, FieldConstants.ampScoringPose.getRotation()));
          path.preventFlipping = true;
          return AutoBuilder.followPath(path);
        },
        Collections.emptySet());
  }
}
