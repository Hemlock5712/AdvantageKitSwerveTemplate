package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * The DriveController class represents a controller for the robot's drive system. It provides
 * methods to control the heading and drive mode of the robot.
 */
public class DriveController {
  private Optional<Supplier<Rotation2d>> headingSupplier = Optional.empty();
  private Supplier<Pose2d> poseSupplier = Pose2d::new;
  private DriveModeType driveModeType = DriveModeType.SPEAKER;

  /**
   * Sets the heading supplier that provides the desired heading for the robot.
   *
   * @param headingSupplier The supplier that provides the desired heading.
   */
  public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = Optional.of(headingSupplier);
  }

  /**
   * Sets the pose supplier that provides the current pose of the robot.
   *
   * @param poseSupplier The supplier that provides the current pose.
   */
  public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  /**
   * Checks if the heading is being controlled.
   *
   * @return True if the heading is being controlled, false otherwise.
   */
  public boolean isHeadingControlled() {
    return this.headingSupplier.isPresent();
  }

  /**
   * Gets the current drive mode.
   *
   * @return The supplier that provides the current drive mode.
   */
  public Supplier<DriveModeType> getDriveModeType() {
    return () -> this.driveModeType;
  }

  /**
   * Gets the current heading angle.
   *
   * @return The supplier that provides the current heading angle.
   */
  public Supplier<Rotation2d> getHeadingAngle() {
    return headingSupplier.get();
  }

  /**
   * Sets the drive mode.
   *
   * @param driveModeType The drive mode to set.
   */
  public void setDriveMode(DriveModeType driveModeType) {
    this.driveModeType = driveModeType;
    updateHeading();
  }

  /** Toggles the drive mode between AMP and SPEAKER. */
  public void toggleDriveMode() {
    if (getDriveModeType().get() == DriveModeType.AMP) {
      setDriveMode(DriveController.DriveModeType.SPEAKER);
    } else {
      setDriveMode(DriveController.DriveModeType.AMP);
    }
  }

  /** Enables heading control based on the current drive mode. */
  public void enableHeadingControl() {
    if (this.driveModeType == DriveModeType.AMP) {
      enableAmpHeading();
    } else {
      enableSpeakerHeading();
    }
  }

  /** Disables heading control (heading control is disabled). */
  public void disableHeadingControl() {
    this.headingSupplier = Optional.empty();
  }

  /** Updates the heading if it is being controlled. */
  private void updateHeading() {
    if (isHeadingControlled()) {
      enableHeadingControl();
    }
  }

  /** Turns on heading control and sets the heading to AMP mode (90 degrees). */
  private void enableAmpHeading() {
    setHeadingSupplier(() -> Rotation2d.fromDegrees(90));
  }

  /** Turns on heading control and sets the heading to SPEAKER mode. */
  private void enableSpeakerHeading() {
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

  /** Possible Drive Modes. */
  public enum DriveModeType {
    AMP,
    SPEAKER,
  }
}
