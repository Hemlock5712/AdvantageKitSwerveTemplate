package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

/**
 * The DriveController class represents a controller for the robot's drive system. It provides
 * methods to control the heading and drive mode of the robot.
 */
public class DriveController {
  @Getter private Optional<Supplier<Rotation2d>> headingSupplier = Optional.empty();
  @Setter private Supplier<Pose2d> poseSupplier = Pose2d::new;

  /**
   * Sets the heading supplier that provides the desired heading for the robot.
   *
   * @param headingSupplier The supplier that provides the desired heading.
   */
  public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = Optional.of(headingSupplier);
  }

  /** Disables heading control (heading control is disabled). */
  public void disableHeadingControl() {
    this.headingSupplier = Optional.empty();
  }

  /** Turns on heading control and sets the heading to AMP mode (90 degrees). */
  public void enableAmpHeading() {
    setHeadingSupplier(() -> Rotation2d.fromDegrees(90));
  }

  /** Turns on heading control and sets the heading to SPEAKER mode. */
  public void enableSpeakerHeading() {
    setHeadingSupplier(
        () ->
            new Rotation2d(
                poseSupplier.get().getX()
                    - AllianceFlipUtil.apply(
                            FieldConstants.Speaker.centerSpeakerOpening.getTranslation())
                        .getX(),
                poseSupplier.get().getY()
                    - AllianceFlipUtil.apply(
                            FieldConstants.Speaker.centerSpeakerOpening.getTranslation())
                        .getY()));
  }

  private void enableStageHeading() {
    setHeadingSupplier(
        () -> {
          int closestChainAprilTagID;
          double targetAngleDegrees;
          // If the alliance is red
          if (DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            closestChainAprilTagID =
                getIDOfClosestStageAprilTag(FieldConstants.redAllianceStageAprilTagIDs);
            if (closestChainAprilTagID == 13) {
              targetAngleDegrees = 0;
            } else if (closestChainAprilTagID == 11) {
              targetAngleDegrees = 120.0;
            } else {
              targetAngleDegrees = 240.0;
            }

          } else {
            // If the alliance is blue
            closestChainAprilTagID =
                getIDOfClosestStageAprilTag(FieldConstants.blueAllianceStageAprilTagIDs);
            if (closestChainAprilTagID == 14) {
              targetAngleDegrees = -180.0;
            } else if (closestChainAprilTagID == 15) {
              targetAngleDegrees = -60.0;
            } else {
              targetAngleDegrees = 60.0;
            }
          }
          return Rotation2d.fromDegrees(targetAngleDegrees);
        });
  }

  private int getIDOfClosestStageAprilTag(int[] AprilTagIDs) {
    int closestAprilTagID = AprilTagIDs[0];
    Optional<Pose3d> closestPose3d = FieldConstants.aprilTags.getTagPose(closestAprilTagID);
    Pose2d closestPose2d = closestPose3d.get().toPose2d();
    double closestDistance =
        closestPose2d.getTranslation().getDistance(poseSupplier.get().getTranslation());
    for (int aprilTagID : AprilTagIDs) {
      Optional<Pose3d> currentPose = FieldConstants.aprilTags.getTagPose(aprilTagID);
      Pose2d currentPose2d = currentPose.get().toPose2d();
      double currentDistance =
          currentPose2d.getTranslation().getDistance(poseSupplier.get().getTranslation());
      if (currentDistance < closestDistance) {
        closestDistance = currentDistance;
        closestAprilTagID = aprilTagID;
      }
    }

    return closestAprilTagID;
  }

  public void enableAmpLobbingHeading() {
    setHeadingSupplier(
        () ->
            new Rotation2d(
                poseSupplier.get().getX()
                    - AllianceFlipUtil.apply(FieldConstants.ampLobbingTarget).getX(),
                poseSupplier.get().getY()
                    - AllianceFlipUtil.apply(FieldConstants.ampLobbingTarget).getY()));
  }
}
